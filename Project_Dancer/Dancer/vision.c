/*
 * vision.c
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#include <stdbool.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "camera/po8030.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#include "main.h"
#include "blinking_leds.h"
#include "vision.h"
#include "trajectory.h"


#define RED							(31 << 11)	//0b11111 000000 00000
#define GREEN 						(63 << 5)	//0b00000 111111 00000
#define BLUE							31			//0b00000 000000 11111
enum Ref_color {R, G, B};

//matrix size
#define M							3
#define N							5

#define GRADIENT_MAX 				500

#define IMAGE_BUFFER_SIZE			640

//object color detection
#define NB_SAMPLES_CALIB				20
#define WIDTH_SAMPLES_CALIB			20
#define START_CALIB					((IMAGE_BUFFER_SIZE-WIDTH_SAMPLES_CALIB)/2 ) //always center the zone
enum Obj_or_back	 					{OBJ, BACK};

#define TOF_MAX_DIST					90
#define TOF_MIN_DIST					50
#define MOVE_TRESH					10
#define DIST_MEAN_RANGE				10

#define D_LENS						772.55f //distance between lense and sensor[pix]
enum Vision_init_state 				{WAIT_OBJECT, CALIB, INIT_DONE};
enum Line_detector_state 			{SEARCH_BEGIN, SEARCH_END, FINISHED};
#define DETECT_TRESH					60
#define MIN_OBJ_SIZE 				40
#define MAX_DELTA_SIZE				30 //maximum variation of size between two samples

#define MOV_AVRG_SIZE				20
#define TRAJ_SCALE					3


//values obtained after moving average, used by trajectoire.c
static float hor_dist = 0;
static float real_dist = 0;

static uint16_t size_obj_mm = 0;
static uint16_t distance_mm_calib = 0;

static bool active = true;



//detection functions
void create_image(uint8_t* image, uint16_t size, uint8_t* img_buff_ptr, float w_r, float w_g, float w_b);
bool dist_measure (uint8_t* image, uint16_t size, bool first_call, float* real_dist_ptr, float* hor_dist_ptr);
float mov_avrg(float value, float* val_tab, uint8_t* oldest_val_ptr);

//calibration functions
void vision_init (uint8_t* image, uint16_t size, uint8_t* img_buff_ptr, float* w_r_ptr, float* w_g_ptr, float* w_b_ptr);
void calib_colors(uint8_t* r_ptr, uint8_t* g_ptr, uint8_t* b_ptr, bool obj_or_back);
bool compute_weights (uint8_t r_obj, uint8_t g_obj, uint8_t b_obj, uint8_t r_back, uint8_t g_back, uint8_t b_back,float* w_r_ptr, float* w_g_ptr, float* w_b_ptr);

//linear algebra functions
int32_t det_3_3 (float matrix [M][N]);
void do_gauss (float matrix [M][N]);
void exchange_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b);
void subtract_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b, float factor);


static BSEMAPHORE_DECL(image_ready_sem, TRUE);


//Thread adapté du cours MICRO-315 (TP4_CamReg), F. Mondada, E. Ferragni
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 240 + 241
    po8030_advanced_config(FORMAT_RGB565, 0, 240, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

    //augment brightness and disable auto white balance for better color recognition
    po8030_set_brightness(64);
    	po8030_set_awb(false);

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

	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint8_t* img_buff_ptr = NULL;
	float w_r = 0; float w_g = 0; float w_b = 0; //weights used to detect the desired color in the image

	chBSemWait(&rec_traj_ready_sem);
	vision_init (image,  IMAGE_BUFFER_SIZE, img_buff_ptr, &w_r, &w_g, &w_b);

	//Instant values of distances
	float inst_real_dist = 0;
	float inst_hor_dist = 0;

	//Tabs for moving average of RD and HD
	float mov_avrg_r_tab [MOV_AVRG_SIZE] = {0};
	uint8_t oldest_val_r = 0;
	float mov_avrg_h_tab [MOV_AVRG_SIZE] = {0};
	uint8_t oldest_val_h = 0;

	uint8_t avrg_count = 0;

	bool prev_active = active;

	while(1){
		if(active){

			if (!prev_active) prev_active = true;
			//waits until an image has been captured
			chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();
			//transform the image using the weights
			create_image(image, IMAGE_BUFFER_SIZE, img_buff_ptr, w_r, w_g, w_b);


			if(dist_measure(image, IMAGE_BUFFER_SIZE, false, &inst_real_dist, &inst_hor_dist)){

				//do moving average with iterative method
				real_dist += mov_avrg(inst_real_dist, mov_avrg_r_tab, &oldest_val_r);
				hor_dist += mov_avrg(inst_hor_dist, mov_avrg_h_tab, &oldest_val_h);

				//verify that all the value of the average tab were initialized before sending the first averaged value
				if(avrg_count < MOV_AVRG_SIZE-1) avrg_count++;
				else signal_dist_ready_sem();
			}
			chThdSleepMilliseconds(10);
		}
		else {
			if (prev_active){
				//reset the moving average for the next time the loop is active
				for(uint8_t i = 0; i < MOV_AVRG_SIZE; i++){
					mov_avrg_r_tab [i] = 0;
					mov_avrg_h_tab [i] = 0;
				}
				oldest_val_r = 0;
				oldest_val_h = 0;
				avrg_count = 0;
			}
			prev_active = false;
			chThdSleepMilliseconds(100);
		}
	}
}


//*************** high level: public functions ***************

float get_real_dist_mm(void){
	return real_dist*TRAJ_SCALE;
}


float get_hor_dist_mm(void){
	return hor_dist*TRAJ_SCALE;
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO+1, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO+1, CaptureImage, NULL);
}


void signal_rec_traj_sem(void){
	chBSemSignal(&rec_traj_ready_sem);
}

void vision_set_active(bool val){
	active = val;
}



//*************** intermediate level: object recognition functions ***************

float mov_avrg(float value, float* val_tab, uint8_t* oldest_val_ptr){
	float diff = (value-val_tab[*oldest_val_ptr])/MOV_AVRG_SIZE;
	val_tab[*oldest_val_ptr	] = value;
	(*oldest_val_ptr) ++;
	*oldest_val_ptr= (*oldest_val_ptr)%MOV_AVRG_SIZE;
	return diff;
}


void create_image(uint8_t* image, uint16_t size, uint8_t* img_buff_ptr, float w_r, float w_g, float w_b){
	for(uint16_t i = 0; i < size; i++){

		//extract colors
		//multiply red and blue (5bit) by 2 to have the same range as green (6 bit)
		uint16_t rgb = ((img_buff_ptr[2*i] << 8) + img_buff_ptr[2*i+1]);
		uint8_t red = 2*((rgb & RED) >> 11);
		uint8_t green = (rgb & GREEN) >> 5;
		uint8_t blue = 2*(rgb & BLUE);

		//compute f: R3->R1 for each pixel, avoid overflow, store new value in image[]
		float f_value = w_r*red+w_g*green+w_b*blue;
		if (f_value < 0) f_value = 0;
		else if(f_value > 255) f_value = 255;
		image[i] = f_value;
	}
}


bool dist_measure (uint8_t* image, uint16_t size, bool first_call, float* real_dist_ptr, float* hor_dist_ptr){

	//starts by searching for the beginning of the object
	uint8_t state = SEARCH_BEGIN;

	uint16_t size_obj_pix = 0;
	int16_t hor_pos_pix = 0;

	//start and stop of the vertical object between 0 and <size>
	uint16_t begin = 0;
	uint16_t end = 0;

	//mean value of the whole image
	float mean = 0.0;
	for(uint16_t k = 0; k < size; k++){
		mean += image[k];
	}
	mean /= size;

	for (uint16_t i = 0; i < size; i ++){

		switch(state){

			case SEARCH_BEGIN :

				if (image[i]-mean > DETECT_TRESH){
					begin = i;
					state = SEARCH_END;
				}
				break;

			case SEARCH_END :

				if (image[i]-mean < DETECT_TRESH){
					end = i;
					size_obj_pix = end-begin;

					//ignore if object is too small, may be a glitch or an unwanted reflect
					if (size_obj_pix < MIN_OBJ_SIZE){
						begin = 0;
						end = 0;
						state = SEARCH_BEGIN;
					}

					//found an object
					else {
						hor_pos_pix = (size_obj_pix - size)/2 + begin;
						state = FINISHED;
					}
				}
				break;

			case FINISHED :

				break;
		}
	}

	//error: no object found or object not entirely visible
	if (state != FINISHED) return false;

	if (first_call){
		float size_dist_ratio = (float)size_obj_pix/D_LENS;
		size_obj_mm = distance_mm_calib*(size_dist_ratio/(1-size_dist_ratio/2));
	}
	else {
		*hor_dist_ptr = (size_obj_mm*hor_pos_pix)/size_obj_pix;
		*real_dist_ptr = (size_obj_mm*sqrt(hor_pos_pix*hor_pos_pix+D_LENS*D_LENS))/size_obj_pix;
	}

	return true;
}




//*************** intermediate level: calibration functions ***************

void vision_init (uint8_t* image, uint16_t size, uint8_t* img_buff_ptr, float* w_r_ptr, float* w_g_ptr, float* w_b_ptr){

	//background identification
	uint8_t r_back, g_back, b_back, r_obj, g_obj, b_obj; //average color values
	calib_colors(&r_back, &g_back, &b_back, BACK);

	//Ready for object identification
	VL53L0X_start();
	uint16_t tof_dist, ref_dist;
	uint32_t mean = 0;
	uint8_t state = WAIT_OBJECT;

	while (state != INIT_DONE){
		switch(state){

			case WAIT_OBJECT :

				led_animation(WAIT_OBJECT_LED);

				for(uint8_t i = 0; i < DIST_MEAN_RANGE; i++){
					uint16_t curent = VL53L0X_get_dist_mm();
					mean += curent;
					chThdSleepMilliseconds(100);
				}
				mean /= DIST_MEAN_RANGE;

				if(mean < TOF_MAX_DIST && mean > TOF_MIN_DIST){
					state = CALIB;
					ref_dist = mean;
				}

				mean = 0;
				chThdSleepMilliseconds(100);
				break;

			case CALIB :

				//control if the user moved
				tof_dist = VL53L0X_get_dist_mm();
				if(abs(tof_dist-ref_dist) > MOVE_TRESH){
					led_animation(ERROR2_LED);
					state = WAIT_OBJECT;
					break;
				}
				led_animation(SUCCESS2_LED);

				 // measure object color then try to compute weights
				calib_colors(&r_obj, &g_obj, &b_obj, OBJ);
				if(!compute_weights (r_obj, g_obj, b_obj, r_back, g_back, b_back, w_r_ptr, w_g_ptr, w_b_ptr)){
					led_animation(TOO_DARK);
					state = WAIT_OBJECT;
					break;
				}

				//measure precisely distance and control if the user moved
				for(uint8_t i = 0; i < 5*DIST_MEAN_RANGE; i++){
					uint16_t curent = VL53L0X_get_dist_mm();
					mean += curent;
					chThdSleepMilliseconds(100);
				}
				distance_mm_calib = mean/(5*DIST_MEAN_RANGE);

				if(abs(distance_mm_calib-ref_dist) > MOVE_TRESH){
					led_animation(ERROR2_LED);
					state = WAIT_OBJECT;
					break;
				}

			  	//waits until an image has been captured, read it and apply weights
				chBSemWait(&image_ready_sem);
				img_buff_ptr = dcmi_get_last_image_ptr();
				create_image(image, size, img_buff_ptr, *w_r_ptr, *w_g_ptr, *w_b_ptr);


				//try to see the object and compute its true size in mm
				if(!dist_measure(image, size, true, NULL, NULL)){
					led_animation(ERROR1_LED);
					state = WAIT_OBJECT;
					break;
				}

				state = INIT_DONE;
				chThdSleepMilliseconds(100);
				break;
		}
	}

	led_animation(SUCCESS2_LED);
	VL53L0X_stop();
}


void calib_colors(uint8_t* r_ptr, uint8_t* g_ptr, uint8_t* b_ptr, bool obj_or_back){

	uint8_t* img_buff_ptr;
	uint32_t sum_r = 0; uint32_t sum_g = 0; uint32_t sum_b = 0;
	uint16_t start, width; //start and width of space average

	if (obj_or_back == OBJ){
		//only a small portion
		start = START_CALIB;
		width = WIDTH_SAMPLES_CALIB;
	}
	else{
		//whole image
		start = 0;
		width = IMAGE_BUFFER_SIZE;
	}

	//sum with respect to time
	for(uint8_t i = 0; i < NB_SAMPLES_CALIB; i++){

		chBSemWait(&image_ready_sem);
		img_buff_ptr = dcmi_get_last_image_ptr();

		//sum with respect to space, the object is supposed to be monochrome
		for(uint16_t j = start; j < start+width; j++){

			//multiply red and blue (5bit) by 2 to have the same range as green (6 bit)
			uint16_t rgb = ((img_buff_ptr[2*j] << 8) + img_buff_ptr[2*j+1]);
			sum_r += 2*((rgb & RED) >> 11);
			sum_g += (rgb & GREEN) >> 5;
			sum_b += 2*(rgb & BLUE);
		}
	}

	*r_ptr = sum_r/(NB_SAMPLES_CALIB*width);
	*g_ptr = sum_g/(NB_SAMPLES_CALIB*width);
	*b_ptr = sum_b/(NB_SAMPLES_CALIB*width);
}


bool compute_weights (uint8_t r_obj, uint8_t g_obj, uint8_t b_obj, uint8_t r_back, uint8_t g_back, uint8_t b_back, float* w_r_ptr, float* w_g_ptr, float* w_b_ptr){

	uint8_t goal_val_obj = 255;
	uint8_t goal_val_back = 0;

	//add a line to the matrix to solve with Gauss and change last line until the matrix is invertible
	//fourth column represents goal values of F, fifth column is for tracking reference color
	float sys[M][N] = {	{r_obj,  g_obj,  b_obj,  goal_val_obj,  0},
						{r_back, g_back, b_back, goal_val_back, 0},
						{0,      0,      1,      0,             1} };

	//by default, the 1 on last line is placed in third position and will represent blue when solving
	uint8_t ref_color = B;

	while (det_3_3(sys) == 0){
		if(sys[2][0] == 1) return false; // tried every position for the 1, error: dimension of system is less than 3

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
	if (((*w_r_ptr)*(*w_r_ptr)+(*w_g_ptr)*(*w_g_ptr)+(*w_b_ptr)*(*w_b_ptr)) > GRADIENT_MAX) return false; // object is either too dark or too similar to background
	else return true;
}




//*************** low level: linear algebra functions ***************

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
