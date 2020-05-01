
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "motors.h"

#include "main.h"
#include "trajectoire.h"
#include "sound.h"
#include "vision.h"
#include "user_button.h"

#include "chprintf.h"

enum Main_states {WAIT, REC_SONG, REC_TRAJ, SHAZAM, DANCE};


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



int main(void){

	halInit();
    chSysInit();
    mpu_init();

    //starts trajectory control and motors
    motors_init();
    trajectoire_start();

    //start user button
    button_start();
    bool double_click = false;

    //start communication with computer
    usb_start();
    serial_start();
    chThdSleepMilliseconds(2000);

    //starts vision
    dcmi_start();
	po8030_start();
	process_image_start();

	//Main finite-state machine
	uint8_t main_state = WAIT;



    /* Infinite loop. */
    while (1) {
//    	//waits 1 second
       chThdSleepMilliseconds(1000);

		switch (main_state){

			case WAIT:
				double_click = wait_click();
//				if(double_click) main_state = REC_SONG;
//				else main_state = SHAZAM;
				main_state = REC_TRAJ;
				break;

			case REC_SONG:
				break;

			case REC_TRAJ:
				chprintf((BaseSequentialStream *) &SDU1,"rec_traj\n");
				signal_rec_traj_sem();
				main_state = DANCE;
				break;

			case SHAZAM:
				break;

			case DANCE:
				break;
		}
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
