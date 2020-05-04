/*
 * audio.h
 *
 *  Created on: 25 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#ifndef AUDIO_H_
#define AUDIO_H_

static BSEMAPHORE_DECL(start_sound_rec_sem, TRUE);

void sound_start(void);



#endif /* AUDIO_H_ */
