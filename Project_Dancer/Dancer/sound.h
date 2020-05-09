/*
 * sound.h
 *
 *  Created on: 25 Apr 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#define MAX_MEM_SONG			3//max number of memorized songs/dance

int8_t audio(uint8_t record_state, uint8_t song_count);
void processAudioData(int16_t *data, uint16_t num_samples);



#endif /* AUDIO_H_ */
