/*
 * audio.h
 *
 *  Created on: 25 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#ifndef AUDIO_H_
#define AUDIO_H_


int8_t audio(uint8_t record_state);
void processAudioData(int16_t *data, uint16_t num_samples);



#endif /* AUDIO_H_ */
