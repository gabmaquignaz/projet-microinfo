/*
 * blinking_leds.h
 *
 *  Created on: 2 May 2020
 *      Author: maximepoffet
 */

#ifndef BLINKING_LEDS_H_
#define BLINKING_LEDS_H_

enum Blinking_state {NO_LED, WAIT_LED, REC_LED, SHAZAM_LED, THINKING_LED, ERROR_LED, SUCCESS_LED};

void set_blinking_state(uint8_t state);
void blinking_start(void);

#endif /* BLINKING_LEDS_H_ */
