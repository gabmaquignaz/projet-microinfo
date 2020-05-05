/*
 * user_button.c
 *
 *  Created on: 30 Apr 2020
 *      Author: gabmac
 */

#include <stdbool.h>
#include "hal.h"
#include "button.h"
#include "blinking_leds.h"

#define DOUBLEDELAY		100 //1s


bool wait_click(void){

	bool double_click = false;


	//wait for click
	while(!button_get_state()){
		chThdSleepMilliseconds(100);


	}

	//Wait for release
	while (button_get_state()){
		chThdSleepMilliseconds(100);
	}


	//wait for potential second click
	uint8_t delay_counter = 0;
	while(true){
		delay_counter ++;
		chThdSleepMilliseconds(10);
		if (button_get_state()){
			double_click = true;
			break;
		}
		else if(delay_counter > DOUBLEDELAY) break;
	}

	return double_click;
}
