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

#define RED							(31 << 11)	//0b11111 000000 00000
#define GREEN 						(63 << 5)	//0b00000 111111 00000
#define BLUE							31			//0b00000 000000 11111
enum color {R, G, B};
#define M							3
#define N							5

#define NB_SAMPLES_CALIB				20			//take 20 successive measurements
#define WIDTH_SAMPLES_CALIB			20			//20 pixels-wide zone
#define START_CALIB					((IMAGE_BUFFER_SIZE-WIDTH_SAMPLES_CALIB)/2 )

enum Line_detector_state {SEARCH_BEGIN, SEARCH_END, FINISHED};
#define DETECT_TRESH				10
#define MIN_OBJ_SIZE 			5

static float hor_dist = 0;
static float real_dist = 0;
static float size2dist_conv = 0;
static uint16_t tof_dist_calib_mm = 0;


int32_t det_3_3 (float matrix [M][N]);
void do_gauss (float matrix [M][N]);
void exchange_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b);
void subtract_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b, float factor);

void read_colors(uint8_t* img_buff_ptr, uint16_t i, uint8_t* colors);
bool compute_weights (uint8_t r_obj, uint8_t g_obj, uint8_t b_obj, uint8_t r_back, uint8_t g_back, uint8_t b_back,float* w_r_ptr, float* w_g_ptr, float* w_b_ptr);
bool dist_measure (uint8_t* image, uint16_t size);


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


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	//reference values for obj and background (should be measured)
//	uint8_t r_obj = 50;
//	uint8_t g_obj = 20;
//	uint8_t b_obj = 20;
//
//	uint8_t r_back = 27;
//	uint8_t g_back = 32;
//	uint8_t b_back = 29;

	uint8_t colors_tab [3] = {0}; //contains the three measured values R,G,B
	float w_r, w_g, w_b; //weights used to detect the desired color in the image
	uint8_t r_obj, g_obj, b_obj, r_back, g_back, b_back; //average color values, used for computation of weights
	uint32_t sum_r = 0; uint32_t sum_g = 0; uint32_t sum_b = 0; //used to store the sum for averages
	uint8_t calibration_counter = 0;

    chThdSleepMilliseconds(1000);

	while(1){
	    	//waits until an image has been captured
	        chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			//***calibration***
			if(calibration_counter < NB_SAMPLES_CALIB){
				for(uint16_t i = START_CALIB; i < START_CALIB + WIDTH_SAMPLES_CALIB; i++){
					//average in space, the object is supposed to be monochrome
					read_colors(img_buff_ptr, i, colors_tab);
					sum_r += colors_tab[R];
					sum_g += colors_tab[G];
					sum_b += colors_tab[B];
				}
				calibration_counter ++;
				if(calibration_counter == NB_SAMPLES_CALIB){
					//average in time
					r_obj = sum_r/(NB_SAMPLES_CALIB*WIDTH_SAMPLES_CALIB);
					g_obj = sum_g/(NB_SAMPLES_CALIB*WIDTH_SAMPLES_CALIB);
					b_obj = sum_b/(NB_SAMPLES_CALIB*WIDTH_SAMPLES_CALIB);
					sum_r = 0;
					sum_g = 0;
					sum_b = 0;
					chThdSleepMilliseconds(3000);
				}
			}
			else if (calibration_counter >= NB_SAMPLES_CALIB && calibration_counter < 2*NB_SAMPLES_CALIB){
				for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
					//average in space, the object is supposed to be monochrome
					read_colors(img_buff_ptr, i, colors_tab);
					sum_r += colors_tab[R];
					sum_g += colors_tab[G];
					sum_b += colors_tab[B];
				}
				calibration_counter ++;
				if(calibration_counter == 2*NB_SAMPLES_CALIB){
					//average in time
					r_back = sum_r/(NB_SAMPLES_CALIB*IMAGE_BUFFER_SIZE);
					g_back = sum_g/(NB_SAMPLES_CALIB*IMAGE_BUFFER_SIZE);
					b_back = sum_b/(NB_SAMPLES_CALIB*IMAGE_BUFFER_SIZE);
					sum_r = 0;
					sum_g = 0;
					sum_b = 0;
					compute_weights(r_obj, g_obj, b_obj, r_back, g_back, b_back, &w_r, &w_g, &w_b);
				}
			}
			else {
				for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

					read_colors(img_buff_ptr, i, colors_tab);

					//compute f: R3->R1 for each pixel, avoid overflow, store value in image[]
					float f_value = w_r*colors_tab[R]+w_g*colors_tab[G]+w_b*colors_tab[B];
					if (f_value < 0) f_value = 0;
					else if(f_value > 255) f_value = 255;
					image[i] = f_value;
				}
				//Send the data
				SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
//				chprintf((BaseSequentialStream *) &SDU1,"OBJCET : R = %d, G = %d, B = %d\n"
//														"BACKGROUND : R = %d, G = %d, B = %d\n\n",
//															r_obj, g_obj, b_obj,r_back,g_back,b_back);
			}
	    }
}


//*************** high level interaction functions (visible by other files) ***************

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



//*************** medium level object recognition functions ***************

bool compute_weights (uint8_t r_obj, uint8_t g_obj, uint8_t b_obj, uint8_t r_back, uint8_t g_back, uint8_t b_back, float* w_r_ptr, float* w_g_ptr, float* w_b_ptr){
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
		if(sys[2][0] == 1) return false; // tried every position for the one, error: dimension of system is less than 3

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
	float alpha_one, beta_one, alpha_two, beta_two, wO;
	switch (ref_color){
		case R:
			alpha_one = sys[1][N-1]/sys[1][1]; //green
			beta_one = sys[1][N-2]/sys[1][1]; // green
			alpha_two = sys[2][N-1]/sys[2][2]; //blue
			beta_two = sys[2][N-2]/sys[2][2]; //blue

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			*w_r_ptr = wO;
			*w_g_ptr = wO*alpha_one+beta_one;
			*w_b_ptr = wO*alpha_two+beta_two;
			break;

		case G:
			alpha_one = sys[0][N-1]/sys[0][0]; //red
			beta_one = sys[0][N-2]/sys[0][0]; // red
			alpha_two = sys[2][N-1]/sys[2][2]; //blue
			beta_two = sys[2][N-2]/sys[2][2]; //blue

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			*w_r_ptr = wO*alpha_one+beta_one;
			*w_g_ptr = wO;
			*w_b_ptr = wO*alpha_two+beta_two;
			break;

		case B:
			alpha_one = sys[0][N-1]/sys[0][0]; //red
			beta_one = sys[0][N-2]/sys[0][0]; // red
			alpha_two = sys[1][N-1]/sys[1][1]; //green
			beta_two = sys[1][N-2]/sys[1][1]; // green

			wO = -(alpha_one*beta_one+alpha_two*beta_two)/(1+alpha_one*alpha_one+alpha_two*alpha_two);

			*w_r_ptr = wO*alpha_one+beta_one;
			*w_g_ptr = wO*alpha_two+beta_two;
			*w_b_ptr = wO;
			break;
	}
	return true;
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
	if (!size2dist_conv) size2dist_conv = size_obj_pix*tof_dist_calib_mm;
	else{
		real_dist = size2dist_conv/size_obj_pix;
		hor_dist = (size_obj_pix - size)/2 + begin;
	}
	return true;
}



//*************** low level calibration functions ***************

void read_colors(uint8_t* img_buff_ptr, uint16_t i, uint8_t* colors){
	//multiply red and blue (5bit) by 2 to have the same range as green (6 bit)
	uint16_t rgb = ((img_buff_ptr[2*i] << 8) + img_buff_ptr[2*i+1]);
	colors[R] = 2*((rgb & RED) >> 11);
	colors[G] = (rgb & GREEN) >> 5;
	colors[B] = 2*(rgb & BLUE);
}



//*************** low level linear algebra functions ***************

void do_gauss (float matrix [M][N]){
	//i represents line, j represents columns

	//elimination of lower non-diagonal coefficients
	for(int8_t i = 1; i < M; i++){
		for(uint8_t j = 0; j < i; j++){
			if(matrix[i][j]){
				float factor = matrix[i][j]/matrix[j][j];
				subtract_lines(matrix,j,i,factor);
			}
		}
	}
	//elimination of upper non-diagonal coefficients
	for(int8_t i = M-2 ; i >= 0; i--){
		for(uint8_t j = M-1; j > i; j--){
			if(matrix[i][j]){
				float factor = matrix[i][j]/matrix[j][j];
				subtract_lines(matrix,j,i,factor);
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


void subtract_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b, float factor){
	//subtracts factor*line_a from line_b
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

