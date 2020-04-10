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

	//reference values for obj and background
	uint8_t r_obj = 40;
	uint8_t g_obj = 35;
	uint8_t b_obj = 40;

	uint8_t r_back = 15;
	uint8_t g_back = 20;
	uint8_t b_back = 23;

	//High/low values
	uint8_t high = 255;
	uint8_t low = 0;

	//add a line to the matrix to solve with Gauss and change last line until the matrix is invertible
	//fourth column represents goal values of F, fifth column is for tracking reference color
	float sys[M][N] = {	{r_obj,g_obj,b_obj,high,0},
						{r_back,g_back,b_back,low,0},
						{0,0,1,0,1} };

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
	switch (ref_color){
		case R:
			exchange_lines(sys,0,2);
			if (!(sys[1][1] && sys[2][2])) exchange_lines(sys,1,2);
			break;

		case G:
			exchange_lines(sys,1,2);
			if (!(sys[0][0] && sys[2][2])) exchange_lines(sys,0,2);
			break;

		case B:
			if (!(sys[0][0] && sys[1][1])) exchange_lines(sys,0,1);
			break;
	}

	//Gauss elimination
	do_gauss(sys);

	//computation of linear relations between R,G and B : weight_color_one(two) = alpha_one(two)*weight_ref_color + beta_one(two)
	//computation of wO, the wait of the ref_color so that the gradient of F is minimum
	//computation the weights w_r, w_g, w_b as a function of wO
	float alpha_one, beta_one, alpha_two, beta_two, wO, w_r, w_g, w_b;
	switch (ref_color){
		case R:
			alpha_one = sys[1][N-1]/sys[1][1]; //green
			beta_one = sys[1][N-2]/sys[1][1]; // green
			alpha_two = sys[2][N-1]/sys[2][2]; //blue
			beta_two = sys[2][N-2]/sys[2][2]; //blue

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			w_r = wO;
			w_g = wO*alpha_one+beta_one;
			w_b = wO*alpha_two+beta_two;
			break;

		case G:
			alpha_one = sys[0][N-1]/sys[0][0]; //red
			beta_one = sys[0][N-2]/sys[0][0]; // red
			alpha_two = sys[2][N-1]/sys[2][2]; //blue
			beta_two = sys[2][N-2]/sys[2][2]; //blue

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			w_r = wO*alpha_one+beta_one;
			w_g = wO;
			w_b = wO*alpha_two+beta_two;
			break;

		case B:
			alpha_one = sys[0][N-1]/sys[0][0]; //red
			beta_one = sys[0][N-2]/sys[0][0]; // red
			alpha_two = sys[1][N-1]/sys[1][1]; //green
			beta_two = sys[1][N-2]/sys[1][1]; // green

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			w_r = wO*alpha_one+beta_one;
			w_g = wO*alpha_two+beta_two;
			w_b = wO;
			break;
	}

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

			//Send the data
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
//			chprintf((BaseSequentialStream *) &SDU1,"a1 = %.2f, b1 = %.2f, a2 = %.2f, b2 = %.2f\n"
//													"wR = %.2f, wG = %.2f, wB = %.2f\n",
//													alpha_one, beta_one, alpha_two, beta_two,w_r,w_g,w_b);
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

void do_gauss (float matrix [M][N]){
	//i represents line, j represents columns

	//elimination of lower non-diagonal coefficients
	for(int8_t i = 1; i < M; i++){
		for(uint8_t j = 0; j < i; j++){
			if(matrix[i][j]){
				float factor = matrix[i][j]/matrix[j][j];
				substract_lines(matrix,j,i,factor);
			}
		}
	}
	//elimination of upper non-diagonal coefficients
	for(int8_t i = M-2 ; i >= 0; i--){
		for(uint8_t j = M-1; j > i; j--){
			if(matrix[i][j]){
				float factor = matrix[i][j]/matrix[j][j];
				substract_lines(matrix,j,i,factor);
			}
		}
	}
}

void exchange_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b){
	if(line_a < M && line_b < M && line_a != line_b){
		float temporary_line [N] = {0};
		for(uint8_t i = 0; i < N; i ++){
			//exchange lines a and b
			temporary_line [i] = matrix[line_a][i];
			matrix[line_a][i] = matrix[line_b][i];
			matrix[line_b][i] = temporary_line[i];
		}
	}
}

void substract_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b, float factor){
	//substracts factor*line_a from line_b
	if(line_a < M && line_b < M && line_a != line_b){
		for(uint8_t i = 0; i < N; i ++){
			matrix[line_b][i] -= factor*matrix[line_a][i];
		}
	}
}

int32_t det_3_3 (float matrix [M][N]){
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

