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

//fifty samples composed of the five stronger frequencies at a given time are stored
//frequency are refered to by their index i in the micOutput array, conversion to [Hz] is useless
#define NB_SAMPLES			50
#define SAMPLE_SIZE 			5
#define OFFSET_MAX			20 //At least (NB_SAMPLES-OFFSET_MAX) must be used for a match
#define MATCH_F_TOL 			2 // frequency difference tolerance to count as a match
#define MATCH_TRESH			30

#define WAIT_FFT 			2 //one in N set of 1024 samples is used
#define PEAK_H_MIN			1200


void processAudioData(int16_t *data, uint16_t num_samples);

float match_song (uint16_t measured_song [NB_SAMPLES][SAMPLE_SIZE], uint16_t ref_song [NB_SAMPLES][SAMPLE_SIZE]);
uint8_t match_sample (uint16_t measured_sample [SAMPLE_SIZE], uint16_t ref_sample [SAMPLE_SIZE]);
uint8_t fit_value (uint8_t fit_ratio);

void doFFT_optimized(uint16_t size, float* complex_buffer);

void extract_freq_id (float* micOutput, uint16_t* sample);
uint8_t find_replace (float* micOutput, uint16_t* sample);
void do_bbl_sort(uint16_t* sample);




static BSEMAPHORE_DECL(FFT_ready_sem, TRUE);

static float micFront_cmplx_input[2 * FFT_SIZE];
static float micFront_output[FFT_SIZE]; //Array containing the computed magnitude of the complex numbers

//???taille WA???
static THD_WORKING_AREA(waCaptureSound, 2048);
static THD_FUNCTION(CaptureSound, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

   uint16_t ref_song [NB_SAMPLES][SAMPLE_SIZE] = {0};
   uint16_t measured_song [NB_SAMPLES][SAMPLE_SIZE] = {0};

   mic_start(&processAudioData);

   chThdSleepMilliseconds(5000);
   	  chprintf((BaseSequentialStream *) &SDU1, "3\n");
   	  chThdSleepMilliseconds(1000);
   	  chprintf((BaseSequentialStream *) &SDU1, "2\n");
   	  chThdSleepMilliseconds(1000);
   	  chprintf((BaseSequentialStream *) &SDU1, "1\n");
   	  chThdSleepMilliseconds(1000);
   	  chprintf((BaseSequentialStream *) &SDU1, "GO\n");

   for (uint8_t i = 0; i < NB_SAMPLES; i++){
	   chBSemWait(&FFT_ready_sem);
	   extract_freq_id(micFront_output, ref_song[i]);
   }

//   for (uint8_t i = 0; i < NB_SAMPLES; i++){
//	   for(uint8_t j = 0; j < SAMPLE_SIZE; j++){
//   		   chprintf((BaseSequentialStream *) &SDU1, "%d ", ref_song[i][j]);
//   	   }
//	   chprintf((BaseSequentialStream *) &SDU1, "\n");
//   }
//   chprintf((BaseSequentialStream *) &SDU1, "\n\n\n");

   while (1) {

	   for (uint8_t i = 0; i < NB_SAMPLES; i++){
	  	   for(uint8_t j = 0; j < SAMPLE_SIZE; j++){
	  	measured_song[i][j] = 0;
	  	   }
	    }
	   chThdSleepMilliseconds(5000);
	  chprintf((BaseSequentialStream *) &SDU1, "3\n");
	  chThdSleepMilliseconds(1000);
	  chprintf((BaseSequentialStream *) &SDU1, "2\n");
	  chThdSleepMilliseconds(1000);
	  chprintf((BaseSequentialStream *) &SDU1, "1\n");
	  chThdSleepMilliseconds(1000);
	  chprintf((BaseSequentialStream *) &SDU1, "GO\n");

	   for (uint8_t i = 0; i < NB_SAMPLES; i++){
	   	   chBSemWait(&FFT_ready_sem);
	   	   extract_freq_id(micFront_output, measured_song[i]);
	   }

//	   for (uint8_t i = 0; i < NB_SAMPLES; i++){
//		   for(uint8_t j = 0; j < SAMPLE_SIZE; j++){
//				   chprintf((BaseSequentialStream *) &SDU1, "%d ", measured_song[i][j]);
//			   }
//		   chprintf((BaseSequentialStream *) &SDU1, "\n");
//	   }
	   float match = match_song(measured_song, ref_song);
	   if (match > MATCH_TRESH) chprintf((BaseSequentialStream *) &SDU1, "SUCCESS ");
	   else chprintf((BaseSequentialStream *) &SDU1, "FAILED ");
	   chprintf((BaseSequentialStream *) &SDU1, "(%.2f)\n", match);

   }
}



//*************** High level interaction functions (visible by other files) ***************

void sound_start(void){
	chThdCreateStatic(waCaptureSound, sizeof(waCaptureSound), NORMALPRIO, CaptureSound, NULL);
}



//*************** Intermediate level: Match testing functions ***************

//computes a match score between a reference song and a measured song
float match_song (uint16_t measured_song [NB_SAMPLES][SAMPLE_SIZE], uint16_t ref_song [NB_SAMPLES][SAMPLE_SIZE]){
	float best_avrg_match = 0;
	//Try match with different offsets, equivalent to a correlation between two discrete signals
	for(int8_t offset = -OFFSET_MAX; offset <= OFFSET_MAX; offset++){

		int8_t start, end;
		if (offset < 0){
			start = 0;
			end = NB_SAMPLES+offset;
		}
		else{
			start = offset;
			end = NB_SAMPLES;
		}

		float mean = 0;
		for(int8_t i = start; i < end; i++){
			mean += match_sample(measured_song[i-offset], ref_song[i]);
		}
		mean /= (end-start);
//		chprintf((BaseSequentialStream *) &SDU1, "%d)  %.2f\n", offset, mean);
		if(mean > best_avrg_match) best_avrg_match = mean;
	}
	return best_avrg_match;
}

uint8_t match_sample (uint16_t measured [SAMPLE_SIZE], uint16_t ref [SAMPLE_SIZE]){

	//verify that the reference is not full of zeros (filtered noise)
	bool empty_ref = true;
	for(int8_t i = 0; i < SAMPLE_SIZE; i++){
		if(ref[i]) empty_ref = false;
	}
	if(empty_ref) return 0;


	uint8_t match_counter = 0;
	uint8_t best_match = 0;
	bool used [SAMPLE_SIZE] = {false};

	for(int8_t i = 0; i < SAMPLE_SIZE; i++){
		for(int8_t j = 0; j < SAMPLE_SIZE; j++){

			//find a starting value for best_match that hasn't been used yet
			while(used[best_match]){
				best_match ++;
				best_match = best_match % SAMPLE_SIZE;
			}
			//find the frequency in ref closest to measure[i]
			if(abs(measured[i]-ref[j]) < abs(measured[i]-ref[best_match]) && !used[j]) best_match = j;
		}

		if(abs(measured[i]-ref[best_match]) <= MATCH_F_TOL){
			//it's a match !
			match_counter ++;
			used[best_match] = true;
		}
	}

	uint8_t fit_ratio = 100*match_counter/SAMPLE_SIZE;
	return fit_value(fit_ratio);
}


//Goodness of fit function, input: fit ratio (0 -> 1) between two samples
//uses a second order polynomial to return a value between 0 and ~150
uint8_t fit_value (uint8_t fit_ratio){
	return (0.015*fit_ratio*fit_ratio-0.03*fit_ratio);
}



//*************** Mic callback ***************

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



//*************** Low level: complex analysis functions ***************

//optimized FFT provided by ARM
void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}



//*************** Low level: frequency recognition functions ***************

//Searches the FFT for peaks and store the five stronger freqs in sample
void extract_freq_id (float* micOutput,  uint16_t* sample){

	uint8_t index_replace_next = 0;

	for(uint16_t i = 1; i < FFT_SIZE/2-1; i++){

		//find a peak and ignore it if it's not high enough
		if((micOutput[i]-micOutput[i-1]) > 0 && (micOutput[i+1]-micOutput[i]) < 0 && micOutput[i] > PEAK_H_MIN){

			//find the next element of the sample that must be replaced
			index_replace_next = find_replace(micOutput, sample);
			//the new peak is higher than the one at index_replace_next or the element to be replaced is still initialized at 0
			if((micOutput[i] > micOutput[sample[index_replace_next]]) || !sample[index_replace_next]) sample[index_replace_next] = i;
		}
	}
}


//finds the frequency of the sample that should be replaced next
//Either a frequency that is still initialized at 0 or the freq that has the smallest peak from the sample
uint8_t find_replace (float* micOutput, uint16_t* sample){
	uint8_t index_smallest = 0;
	for(uint8_t i = 1; i < SAMPLE_SIZE; i++){
		if(!sample[i]) return i; // a freq is still initialized at zero and should be replaced next
		if(micOutput[sample[i]] < micOutput[sample[index_smallest]]) index_smallest = i;
	}
	return index_smallest;

}
