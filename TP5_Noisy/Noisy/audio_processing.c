#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

enum Mic {R,L,B,F};
#define ONE_MIC				(640/4)

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];


static uint16_t counter = 0;
static uint8_t sent = 0;

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void sound_remote(float* data){
	uint16_t max_pos = 0;

	for(uint16_t i = 0; i<FFT_SIZE; i++){
		if (data[i] > data[max_pos]){
			max_pos = i;
		}
	}

	if (max_pos>400){
		left_motor_set_speed(600);
		right_motor_set_speed(600);
	}
	else {
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}

}

void processAudioData(int16_t *data, uint16_t num_samples){

	for(uint8_t i = 0; i < ONE_MIC; i++){

		micLeft_cmplx_input[2*counter] = (float)data[4*i+L]; micLeft_cmplx_input[2*counter+1] = 0;
		micRight_cmplx_input[2*counter] = (float)data[4*i+R]; micRight_cmplx_input[2*counter+1] = 0;
		micBack_cmplx_input[2*counter] = (float)data[4*i+B]; micBack_cmplx_input[2*counter+1] = 0;
		micFront_cmplx_input[2*counter] = (float)data[4*i+F]; micFront_cmplx_input[2*counter+1] = 0;
		counter ++;

		if (counter >= FFT_SIZE) {
			sent ++;

			if (sent >=5){

				doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

				arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

				chBSemSignal(&sendToComputer_sem);

				sent = 0;

//				sound_remote(micFront_output);

			}

			counter = 0;
			break;
		}
	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}


