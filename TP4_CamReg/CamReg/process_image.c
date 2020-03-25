#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <stdbool.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define DEMI_ARRAY_SIZE_G 		3
#define GREEN 					(63 << 5) // 0b0000011111100000
#define SIZE2DIST				360	//pixel*cm


static float distance_cm = 0;

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

	//mesure du temps
	systime_t time_start, time_end;

    while(1){

    		//time_start = chVTGetSystemTime();

        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

		//time_end = chVTGetSystemTime();

		//printf time info to port
		//chprintf((BaseSequentialStream *) &SDU1, "TTTest : %d", delta);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	while(1){
	    	//waits until an image has been captured
	        chBSemWait(&image_ready_sem);
			//gets the pointer to the array filled with the last image in RGB565
			img_buff_ptr = dcmi_get_last_image_ptr();

			for(uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

				// ---Version plus clean---
				//stick the two 8-bit ints together in a 16-bit int,
				//select only green with a mask,
				//shift right and put the value in an 8-bit int

				uint16_t green_select = (((img_buff_ptr[2*i] << 8) + img_buff_ptr[2*i+1]) & GREEN) >> 5;
				image[i] = (uint8_t) green_select;
			}
			//measure line
			line_position (image, IMAGE_BUFFER_SIZE);

			//Send the data
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);

	    }
}

float get_distance_cm(void){
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void line_position (uint8_t* image, uint16_t size){

	bool reading_black = false;
	uint8_t tresh = 5;

	uint16_t black_start = 0;
	uint16_t black_stop = 0;

	uint16_t sum = 0;
	for (uint16_t i = 0; i < size; i ++){
		sum += image[i];
	}

	uint8_t mean = sum/size;

	for (uint16_t i = 0; i < size; i ++){
			if (image[i] < (mean - tresh) && !reading_black){
				reading_black = true;
				black_start = i;
			}
			else if (image[i] > (mean - tresh) && reading_black){
				reading_black = false;
				black_stop = i;

				uint16_t largeur_trait = black_stop-black_start;
				distance_cm = SIZE2DIST/largeur_trait;

				//TEST

				//output information
				//chprintf((BaseSequentialStream *) &SDU1, "Position : %d, Size : %d\n", black_start + largeur_trait/2, largeur_trait );
			}
	}

	return;
}
