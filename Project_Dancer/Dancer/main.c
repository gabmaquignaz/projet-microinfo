
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "usbcfg.h"
#include "motors.h"
#include <audio/microphone.h>
#include "spi_comm.h"

#include "main.h"
#include "trajectoire.h"
#include "sound.h"
#include "vision.h"
#include "user_button.h"
#include "blinking_leds.h"

#include "chprintf.h"



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

static THD_WORKING_AREA(waMainFSM, 256);
static THD_FUNCTION(MainFSM, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //Main finite-state machine
    	uint8_t main_state = WAIT;
    bool double_click = false;

    	//id of the identified song, -1 -> nothing
    	int8_t current_dance = -1;

    	//number of memorized song/dance
    	uint8_t song_count = 0;

	//Infinite loop
	while (true) {
		//waits 1 second
		chThdSleepMilliseconds(1000);

		switch (main_state){

			case WAIT:
				double_click = wait_click();

				if(double_click){
					main_state = RECORD;

				}
				else {
					main_state = SHAZAM;

				}

				break;

			case RECORD:
				chprintf((BaseSequentialStream *) &SD3,"record\n");

				if(song_count == MAX_MEM_SONG) {
					//error: too many songs
					led_animation(ERROR2_LED);
					chprintf((BaseSequentialStream *) &SD3,"too many songs\n");
				}
				else {
					audio(RECORD, song_count);

					//init vision
					if(song_count == 0) signal_rec_traj_sem();

					save_trajectory(song_count);
					led_animation(SUCCESS2_LED);

					song_count ++;
				}
				main_state = WAIT;
				break;

			case SHAZAM:
				chprintf((BaseSequentialStream *) &SD3,"shazam\n");
				if(song_count == 0) {
					//error: zero songs memorized
					led_animation(ERROR2_LED);
					chprintf((BaseSequentialStream *) &SD3,"zero songs memorized\n");
				}
				else{
					current_dance = audio(SHAZAM, song_count);
					if(current_dance == -1) led_animation(ERROR1_LED); //no matching song
					else {
						led_animation(SUCCESS1_LED);
						set_blinking_state(DANCE_LED);
						dance(current_dance);
						set_blinking_state(NO_LED);
					}
				}
				main_state = WAIT;
				break;
		}
	}
}


int main(void){

	halInit();
    chSysInit();
    mpu_init();

    //starts motors
    motors_init();

    //spi communication for user button
    spi_comm_start();

    //start communication with computer
    usb_start();
    serial_start();
    chThdSleepMilliseconds(2000);

    //starts vision for trajectory recognition
    //augment brightness and disable auto white balance for better color recognition
    dcmi_start();
	po8030_start();
	po8030_set_brightness(64);
	po8030_set_awb(false);
	process_image_start();

	//starts mic for song recognition
	mic_start(&processAudioData);

	//start leds
	set_blinking_state(NO_LED);
	blinking_start();

	chThdCreateStatic(waMainFSM, sizeof(waMainFSM), NORMALPRIO, MainFSM, NULL);


	//infinite loop
	while(true){
		chThdSleepMilliseconds(1000);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
