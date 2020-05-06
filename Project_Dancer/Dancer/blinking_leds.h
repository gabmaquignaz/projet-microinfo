/*
 * blinking_leds.h
 *
 *  Created on: 2 May 2020
 *      Author: maximepoffet
 */

#ifndef BLINKING_LEDS_H_
#define BLINKING_LEDS_H_

enum Blinking_state {NO_LED, REC_TRAJ_LED, REC_SOUND_LED,SHAZAM_LED, DANCE_LED, ERROR1_LED,
					ERROR2_LED, SUCCESS1_LED, SUCCESS2_LED, COUNTDOWN, TOO_DARK, WAIT_OBJECT_LED};

static BSEMAPHORE_DECL(led_sem, TRUE);

void set_blinking_state(uint8_t state);
void led_animation(uint8_t state);
void blinking_start(void);

#endif /* BLINKING_LEDS_H_ */
