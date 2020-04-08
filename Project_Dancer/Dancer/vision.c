/*
 * vision.c
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <stdbool.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <vision.h>

#define RED						2*((rgb & (31 << 11)) >> 11)	//0b11111 000000 00000
#define GREEN 					((rgb & (63 << 5)) >> 5)		//0b00000 111111 00000
#define BLUE						2*(rgb & 31) 				//0b00000 000000 11111

#define BGND_NB_SAMPLES			100
#define DIFF_TRESH				10
#define SELECTIVITY				0.7

enum Line_detector_stae {SEARCH_BEGIN, SEARCH_END, FINISHED};
#define DETECT_TRESH				10
#define MIN_OBJ_SIZE 			5

static float hor_dist = 0;
static float real_dist = 0;
static float size2dist_conv = 0;
static uint16_t tof_dist_calib = 0;



void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {


    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	uint16_t background[IMAGE_BUFFER_SIZE] = {0};
	uint8_t background_capture_count = 0;

	//reference values for obj and background
	uint8_t r_obj = 50;
	uint8_t g_obj = 15;
	uint8_t b_obj = 17;

	uint8_t r_back = 20;
	uint8_t g_back = 20;
	uint8_t b_back = 18;

	//High/low values
	uint8_t high = 250;
	uint8_t low = 0;

//	//dim(system) < 2 -> cannot solve system !
//	if((r_obj*g_back)==(r_back*g_obj) && (r_obj*b_back)==(r_back*b_obj) && (g_obj*b_back)==(g_back*b_obj)){
//		//error !!
//	}
//	//solving for 1D space of solutions
//	else if(){
//
//	}



	float alpha_r = -0.1;
	float beta_r = 5.714;
	float alpha_g = -0.8;
	float beta_g = -5.714;

	float wO = -(alpha_r*beta_r+alpha_g*beta_g)/(1+alpha_r*alpha_r+alpha_g*alpha_g);
	float w_r = wO*alpha_r+beta_r;
	float w_g = wO*alpha_g+beta_g;
	float w_b = wO;


	while(1){
	    	//waits until an image has been captured
	        chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();


			for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

				uint16_t rgb = ((img_buff_ptr[2*i] << 8) + img_buff_ptr[2*i+1]);
				float value = w_r*RED+w_b*GREEN+w_b*BLUE;
				if(value <0) value = 0;
				else if(value > 255) value = 255;
				image[i]=value;
			}

			/*
			if (background_capture_count < BGND_NB_SAMPLES){

				//background capture and average
				background_set(background, image, background_capture_count);
				background_capture_count++;
			}
			else {

				//subtracting background from image and measuring distances
				background_ignore(background, image);
				dist_measure(image, IMAGE_BUFFER_SIZE);
			}
			*/
			//Send the data
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
//			chprintf((BaseSequentialStream *) &SDU1,
//						"wR = %.2f, wG = %.2f, wB = %.2f\n",w_r,w_g,w_b);

	    }
}

uint16_t get_real_dist_mm(void){
	return real_dist;
}

uint16_t get_hor_dist_mm(void){
	return hor_dist;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void background_set(uint16_t *background, uint8_t *image, uint8_t counter){
	for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
		background[i] += image[i];
		if (counter >= BGND_NB_SAMPLES-1) background[i] /= BGND_NB_SAMPLES;
	}
}

void background_ignore(uint16_t *background, uint8_t *image){
	for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

		int16_t diff = image[i] - background[i];
		//show diff, correct only background (small diff between registered background), not object (big diff)
		if (1) {
			// center to have maximum dynamic
			diff += 128;

			//saturation instead of overflow
			if (diff < 0) diff = 0;
			else if(diff > 255) diff = 255;

			image[i] = diff;
		}
		else image[i] = 0;
	}
}


bool dist_measure (uint8_t* image, uint16_t size){

	//state of the object detector, starts by searching for the beginning of the object
	uint8_t state = SEARCH_BEGIN;

	//start and stop of the vertical object between 0 and <size>
	uint16_t begin = 0;
	uint16_t end = 0;

	for (uint16_t i = 1; i < size; i ++){
		switch(state){

			case SEARCH_BEGIN :
				if (image[i] - image[i-1] > DETECT_TRESH){
					begin = i;
					state = SEARCH_END;
				}
				break;

			case SEARCH_END :
				if (image[i] - image[i-1] < -DETECT_TRESH){
					if (end-begin < MIN_OBJ_SIZE){
						//error: object too small, may be a glitch or an unwanted reflect
						begin = 0;
						state = SEARCH_BEGIN;
					}
					else{
						end = i;
						state = FINISHED;
					}
				}
				break;

			case FINISHED :
				//error: multiple objects
				if (image[i] - image[i-1] > DETECT_TRESH) return false;
				break;
		}
	}

	//error: no object found or object not entirely visible
	if (state != FINISHED) return false;

	uint16_t size_obj_pix = end-begin;

	//if not initialized yet, computation of the constant for size-to-distance conversion, else compute distance
	if (!size2dist_conv) size2dist_conv = size_obj_pix*tof_dist_calib;
	else{
		real_dist = size2dist_conv/size_obj_pix;
		hor_dist = (size_obj_pix - size)/2 + begin;
	}
	return true;
}

