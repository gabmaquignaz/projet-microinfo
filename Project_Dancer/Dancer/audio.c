/*
 * audio.c
 *
 *  Created on: 25 Apr 2020
 *      Author: Gabriel Maquignaz
 */


#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <stdlib.h>

#include <motors.h>
#include <audio/microphone.h>
#include <arm_math.h>
#include <arm_const_structs.h>

enum Mic {R,L,B,F};
#define NB_SAMPLES_ONE_MIC	160
#define FFT_SIZE 			1024

//fifty samples composed of the five stronger frequencies are stored
//frequency are refered to by their index i in the output array
#define NB_SAMPLES			50
#define FREQ_ID_SIZE 		5



void doFFT_optimized(uint16_t size, float* complex_buffer);
void processAudioData(int16_t *data, uint16_t num_samples);
uint8_t find_smallest (float* micOutput, uint16_t* sample);
void extract_freq_id (float* micOutput,  uint16_t* sample);
int cmpfunc (const void * a, const void * b);



//semaphore
static BSEMAPHORE_DECL(FFT_ready_sem, TRUE);

static float micFront_cmplx_input[2 * FFT_SIZE];
static float micFront_output[FFT_SIZE]; //Array containing the computed magnitude of the complex numbers




static THD_WORKING_AREA(waCaptureSound, 1024);
static THD_FUNCTION(CaptureSound, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

   uint16_t song_data [NB_SAMPLES][FREQ_ID_SIZE] = {0};
   mic_start(&processAudioData);

   while (1) {
	   for (uint8_t i = 0; i < NB_SAMPLES; i++){
		   chBSemWait(&FFT_ready_sem);
		   extract_freq(micFront_output, song_data[i]);
		   chprintf((BaseSequentialStream *) &SDU1, "%d %d %d %d %d\n", song_data[i][0], song_data[i][1], song_data[i][2], song_data[i][3], song_data[i][4]);
	   }
	   chThdSleepMilliseconds(5000);
   }
}






/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*/

void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t counter = 0;
	if (num_samples != 640) //error

	for(uint8_t i = 0; i < NB_SAMPLES_ONE_MIC; i++){
		//Only use front mic
		micFront_cmplx_input[2*counter] = (float)data[4*i+F];
		micFront_cmplx_input[2*counter+1] = 0;
		counter ++;

		if (counter >= FFT_SIZE) {

			doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
			arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
			chBSemSignal(&FFT_ready_sem);
			counter = 0;
			break;
		}
	}
}

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}

void extract_freq_id (float* micOutput,  uint16_t* sample){
	//a sample contains the indexes in the output tab (i.e. the frequency value) of the five highest freqs
	uint8_t index_smallest = 0;

	for(uint16_t i = 0; i < FFT_SIZE/2; i++){

		//find the index of the smallest of the five freqs
		//if its intensity is smaller than that of freq of index i, replace it
		index_smallest = find_smallest(micOutput, sample);

		if (micOutput[i] > micOutput[ sample[index_smallest] ]){
			sample[index_smallest] = i;
		}
	}

	//quick sort the five higher frequencies
	qsort(sample, FREQ_ID_SIZE, sizeof(uint16_t), cmpfunc);
}

uint8_t find_smallest (float* micOutput, uint16_t* sample){
	//a sample contains the indexes in the output tab (i.e. the frequency value) of the five highest freqs
	uint8_t index_smallest = 0;
	uint16_t i;
	for(i = 1; i < FREQ_ID_SIZE; i++){
			if(micOutput[sample[i]] < micOutput[sample[index_smallest]]) index_smallest = i;
	}
	return i;
}


int cmpfunc (const void * a, const void * b){
   return ( *(int*)a - *(int*)b );
}
