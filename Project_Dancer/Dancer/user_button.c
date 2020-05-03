/*
 * user_button.c
 *
 *  Created on: 30 Apr 2020
 *      Author: gabmac
 */

#include <stdbool.h>
#include "hal.h"
#include "button.h"
#include "spi_comm.h"
#include "blinking_leds.h"

#define CLICKDELAY		1000 //0.1s
#define DOUBLEDELAY		10000 //1s


static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        10000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

void button_start(void){
	spi_comm_start(); //start comm for button tracking
	timer12_start();
}


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

	GPTD12.tim->CNT = 0;

	//wait for potential second click
	while(true){
//      chprintf((BaseSequentialStream *) &SDU1, "%d\n", GPTD12.tim->CNT);
		if (button_get_state() && GPTD12.tim->CNT > CLICKDELAY){
			double_click = true;
		}
		if (GPTD12.tim->CNT > DOUBLEDELAY) break;
	}

	return double_click;
}
