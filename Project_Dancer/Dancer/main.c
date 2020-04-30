
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "main.h"
#include "trajectoire.h"
#include "motors.h"
#include "button.h"
#include "leds.h"

#define CLICKDELAY		100
#define DOUBLEDELAY		1000


int main(void)
{
	systime_t time;
	bool doubleclick = false;


    halInit();
    chSysInit();
    mpu_init();
    motors_init();



    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        //chThdSleepMilliseconds(10);

        if(button_get_state()){
        		time = chVTGetSystemTime();
        		doubleclick = false;
        		while(true){
        			if (button_get_state() && chVTGetSystemTime()-time > CLICKDELAY){
        				doubleclick = true;
        			}
        			if (chVTGetSystemTime()-time > DOUBLEDELAY) break;
        		}

        		if (doubleclick){
        			//time to shazam
        			set_body_led(1);
        		}
        		else{
        			//time to save a new song
        			set_led(1, 1);
				set_led(3, 1);
				set_led(5, 1);
				set_led(7, 1);
        		}

        }

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
