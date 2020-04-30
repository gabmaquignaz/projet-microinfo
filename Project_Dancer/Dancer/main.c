
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "main.h"
#include "trajectoire.h"
#include "motors.h"
#include "button.h"
#include "leds.h"
#include "spi_comm.h"



#include <chprintf.h>


#define CLICKDELAY		100000 //0.1s
#define DOUBLEDELAY		1000000 //1s

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}

int main(void)
{
	systime_t time;
	bool doubleclick = false;


    halInit();
    chSysInit();
    mpu_init();
    motors_init();

    spi_comm_start();
    usb_start();
    serial_start();
    timer12_start();



    /* Infinite loop. */
    while (1) {
    		//waits 1 second
//        chThdSleepMilliseconds(500);

        if(button_get_state()){
        		GPTD12.tim->CNT = 0;
        		doubleclick = false;
        		while(true){
        			if (button_get_state() && GPTD12.tim->CNT > CLICKDELAY){
        				doubleclick = true;
        			}
        			if (GPTD12.tim->CNT > DOUBLEDELAY) break;
        		}

        		if (doubleclick){
        			//time to shazam
        			set_body_led(2);
        			chThdSleepMilliseconds(1000);
        			set_body_led(0);
        		}
        		else{
        			//time to save a new song
        			set_led(1, 2);
				set_led(3, 2);
				set_led(5, 2);
				set_led(7, 2);
				chThdSleepMilliseconds(1000);
				set_led(1, 0);
				set_led(3, 0);
				set_led(5, 0);
				set_led(7, 0);

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
