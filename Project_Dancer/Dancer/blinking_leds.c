/*
 * blinking_leds.c
 *
 *  Created on: 2 May 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */
#include <stdint.h>
#include "ch.h"
#include "blinking_leds.h"
#include "leds.h"
#include "main.h"

static uint8_t blinking_state;

static THD_WORKING_AREA(waBlinking, 256);
static THD_FUNCTION(Blinking, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	while (1){
	    switch (blinking_state){
	    		case NO_LED:
	    			chBSemWait(&led_sem);
	    			break;

			case SHAZAM_LED:
				set_led(LED1, 2);
				chThdSleepMilliseconds(250);
				set_led(LED1, 0);
				set_led(LED3, 2);
				chThdSleepMilliseconds(250);
				set_led(LED3, 0);
				set_led(LED5, 2);
				chThdSleepMilliseconds(250);
				set_led(LED5, 0);
				set_led(LED7, 2);
				chThdSleepMilliseconds(250);
				set_led(LED7, 0);
				break;

			case REC_SOUND_LED:
				set_led(LED1, 2);
				set_led(LED5, 2);
				chThdSleepMilliseconds(400);
				set_led(LED1, 0);
				set_led(LED5, 0);
				set_led(LED3, 2);
				set_led(LED7, 2);
				chThdSleepMilliseconds(400);
				set_led(LED3, 0);
				set_led(LED7, 0);
				break;

			case DANCE_LED:
				set_body_led(2);
				chThdSleepMilliseconds(300);
				set_led(LED1, 2);
				chThdSleepMilliseconds(300);
				set_led(LED1, 0);
				set_led(LED3, 2);
				set_led(LED7, 2);
				chThdSleepMilliseconds(300);
				set_body_led(0);
				set_led(LED3, 0);
				set_led(LED7, 0);
				set_led(LED5, 2);
				chThdSleepMilliseconds(300);
				set_led(LED5, 0);
				set_led(LED3, 2);
				set_led(LED7, 2);
				chThdSleepMilliseconds(300);
				set_led(LED1, 2);
				set_led(LED3, 0);
				set_led(LED7, 0);
				chThdSleepMilliseconds(300);
				set_led(LED1, 0);
				break;
	    }

	}

}

void led_animation(uint8_t state){

	switch (state){
		case ERROR1_LED:
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(300);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED5, 0);
			set_led(LED7, 0);
			chThdSleepMilliseconds(300);
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(300);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED5, 0);
			set_led(LED7, 0);
			chThdSleepMilliseconds(300);
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(800);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED5, 0);
			set_led(LED7, 0);
			break;

		case ERROR2_LED:
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(200);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED5, 0);
			set_led(LED7, 0);
			chThdSleepMilliseconds(200);
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(200);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED5, 0);
			set_led(LED7, 0);
			break;

		case SUCCESS1_LED:
			set_body_led(2);
			chThdSleepMilliseconds(300);
			set_body_led(0);
			chThdSleepMilliseconds(300);
			set_body_led(2);
			chThdSleepMilliseconds(300);
			set_body_led(0);
			chThdSleepMilliseconds(300);
			set_body_led(2);
			chThdSleepMilliseconds(800);
			set_body_led(0);
			break;

		case SUCCESS2_LED:
			set_body_led(2);
			chThdSleepMilliseconds(500);
			set_body_led(0);
			break;

		case REC_TRAJ_LED:
			set_led(LED1, 2);
			chThdSleepMilliseconds(100);
			set_led(LED1, 0);
			break;

		case COUNTDOWN:
			set_led(LED3, 2);
			set_led(LED5, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(1000);
			set_led(LED7, 0);
			chThdSleepMilliseconds(1000);
			set_led(LED5, 0);
			chThdSleepMilliseconds(1000);
			set_led(LED3, 0);
			break;

		case TOO_DARK:
			set_led(LED3, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(300);
			set_led(LED3, 0);
			set_led(LED7, 0);
			chThdSleepMilliseconds(300);
			set_led(LED3, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(300);
			set_led(LED3, 0);
			set_led(LED7, 0);
			break;

		case WAIT_OBJECT_LED:
			set_led(LED1, 2);
			set_led(LED3, 2);
			set_led(LED7, 2);
			chThdSleepMilliseconds(300);
			set_led(LED1, 0);
			set_led(LED3, 0);
			set_led(LED7, 0);
			chThdSleepMilliseconds(300);
			break;

	}
}

void set_blinking_state(uint8_t state){
	blinking_state = state;
	if (state != NO_LED) chBSemSignal(&led_sem);
}

void blinking_start(void){
	chThdCreateStatic(waBlinking, sizeof(waBlinking), NORMALPRIO, Blinking, NULL);
	set_blinking_state(NO_LED);
}
