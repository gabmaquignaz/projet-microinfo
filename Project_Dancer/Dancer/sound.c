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

#include <audio/microphone.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <sound.h>

enum Mic {R,L,B,F};

#define FFT_SIZE 			1024

//fifty samples composed of the five stronger frequencies are stored
//frequency are refered to by their index i in the output array
#define NB_SAMPLES			50
#define FREQ_ID_SIZE 		5

#define WAIT_FFT 			1 //one in N set of 1024 samples is used
#define FREQ_MIN_DIST 		5


void processAudioData(int16_t *data, uint16_t num_samples);
void doFFT_optimized(uint16_t size, float* complex_buffer);
uint8_t find_smallest (float* micOutput, uint16_t* sample);
void extract_freq_id (float* micOutput, uint16_t* sample);
void do_bbl_sort(uint16_t* sample);



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

   for (uint8_t i = 0; i < NB_SAMPLES; i++){
	   chBSemWait(&FFT_ready_sem);
	   extract_freq_id(micFront_output, song_data[i]);
	   chprintf((BaseSequentialStream *) &SDU1, "%d %d %d %d %d\n", song_data[i][0], song_data[i][1], song_data[i][2], song_data[i][3], song_data[i][4]);
   }

   while (1) {
	   chThdSleepMilliseconds(1000);
   }
}




void sound_start(void){
	chThdCreateStatic(waCaptureSound, sizeof(waCaptureSound), NORMALPRIO, CaptureSound, NULL);
}

//Fonction adaptée du cours MICRO-315 (TP5_Noisy), F. Mondada, E. Ferragni
void processAudioData(int16_t *data, uint16_t num_samples){
	static uint16_t samples_counter = 0;
	static uint8_t wait_counter = 0;

	for(uint16_t i = 0; i < num_samples; i+=4){
		//Only use front mic
		micFront_cmplx_input[2*samples_counter] = (float)data[i+F];
		micFront_cmplx_input[2*samples_counter+1] = 0;

		samples_counter ++;

		if (samples_counter >= FFT_SIZE) {

			wait_counter ++;

			if (wait_counter >= WAIT_FFT){
				doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
				arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
				chBSemSignal(&FFT_ready_sem);
				wait_counter = 0;
			}

			samples_counter = 0;
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

		//found a frequency that has higher intensity than the fifth highest found yet
		if (micOutput[i] > micOutput[ sample[index_smallest] ]){

			//if the new frequency is too close to one already existing, ignore it
			bool same_freq = false;
			for(uint8_t j = 0; j < FREQ_ID_SIZE; j++){
				if(i-sample[j] < FREQ_MIN_DIST) same_freq = true;
			}
			if(!same_freq) sample[index_smallest] = i;

		}
	}

	//sort the frequency in ascending order
	do_bbl_sort(sample);
}

void do_bbl_sort(uint16_t* sample){
	//sort the five freqs by peak value
	for(uint8_t i = 0; i < FREQ_ID_SIZE-1; i++){
		for(uint8_t j = 0 ; j < FREQ_ID_SIZE-i-1; j++){
			if(sample[j] > sample[j+1]){
				uint16_t swap = sample[j];
				sample[j] = sample[j+1];
				sample[j+1] = swap;
			}
		}
	}
}

uint8_t find_smallest (float* micOutput, uint16_t* sample){
	//a sample contains the indexes in the output tab (i.e. the frequency value) of the five highest freqs
	uint8_t index_smallest = 0;
	uint16_t i;
	for(i = 1; i < FREQ_ID_SIZE; i++){
			if(micOutput[sample[i]] < micOutput[sample[index_smallest]]) index_smallest = i;
	}
	return index_smallest;

}
