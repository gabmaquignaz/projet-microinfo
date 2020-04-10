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

#define RED						(2*((rgb & (31 << 11)) >> 11))	//0b11111 000000 00000
#define GREEN 					((rgb & (63 << 5)) >> 5)		//0b00000 111111 00000
#define BLUE						(2*(rgb & 31))				//0b00000 000000 11111
enum color {R, G, B};

#define BGND_NB_SAMPLES			100
#define DIFF_TRESH				10
#define SELECTIVITY				0.7

enum Line_detector_state {SEARCH_BEGIN, SEARCH_END, FINISHED};
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

	//augment the matrix to solve with Gauss and change last line until the matrix is invertible
	//fourth column is for tracking of reference color on the other lines
	float sys[3][4] = {	{r_obj,g_obj,b_obj,0},
						{r_back,g_back,b_back,0},
						{0,0,1,1} };

	uint8_t ref_color = B; //by default, the 1 on last line is placed in third position and will represent blue when solving

	while (det_3_3(sys) == 0){
		if(sys[2][0] == 1) {} // tried every position for the one, error: dimension of system is less than 3

		else if(sys[2][2] == 1){
			sys[2][2] = 0;
			sys[2][1] = 1;
			ref_color = G;
		}
		else if(sys[2][1] == 1){
			sys[2][1] = 0;
			sys[2][0] = 1;
			ref_color = R;
		}
	}

	//diagonal coefficients must be non-zero for Gauss algorithm, exchange lines when needed
	if(sys[0][0] == 0){
		if(sys[1][0] == 1) exchange_lines(sys,0,1);
		else exchange_lines(sys,0,2);
	}
	else if(sys[1][1] == 0){
		exchange_lines(sys,1,2);
	}
	//GAUSS



	float alpha_one = -0.1;
	float beta_one = 5.714;
	float alpha_two = -0.8;
	float beta_two = -5.714;

	float wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);
	float w_r = wO*alpha_one+beta_one;
	float w_g = wO*alpha_two+beta_two;
	float w_b = wO;


	while(1){
	    	//waits until an image has been captured
	        chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();


			for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

				uint16_t rgb = ((img_buff_ptr[2*i] << 8) + img_buff_ptr[2*i+1]);

				float value = w_r*RED+w_g*GREEN+w_b*BLUE;
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

void exchange_lines(float matrix [3][4], uint8_t line_a, uint8_t line_b){
	if(line_a < 3 && line_b < 3 && line_a != line_b){
		float temporary_line [4] = {0};
		for(uint8_t i = 0; i < 4; i ++){
			//exchange lines a and b
			temporary_line [i] = matrix[line_a][i];
			matrix[line_a][i] = matrix[line_b][i];
			matrix[line_b][i] = temporary_line[i];
		}
	}
}

int32_t det_3_3 (float matrix [3][4]){
	//before reducing with Gauss' theorem, the matrix only contains ints so the det is an int
	int32_t det = matrix[0][0]*matrix[1][1]*matrix[2][2];
	det += matrix[0][1]*matrix[1][2]*matrix[2][0];
	det += matrix[1][0]*matrix[2][1]*matrix[0][2];
	det -= matrix[0][2]*matrix[1][1]*matrix[2][0];
	det -= matrix[0][1]*matrix[1][0]*matrix[2][2];
	det -= matrix[1][2]*matrix[2][1]*matrix[0][0];
	return det;
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

