
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include "motors.h"
#include <audio/microphone.h>

#include "main.h"
#include "trajectoire.h"
#include "sound.h"
#include "vision.h"
#include "user_button.h"
#include "blinking_leds.h"



static THD_WORKING_AREA(waMainFSM, 256);
static THD_FUNCTION(MainFSM, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    	uint8_t main_state = WAIT;
    bool double_click = false;

    	//id of the identified song, -1 -> nothing
    	int8_t current_song = -1;

    	//current number of memorized song/dance
    	uint8_t song_count = 0;

	//Infinite loop
	while (true) {

		chThdSleepMilliseconds(1000);

		switch (main_state){

			case WAIT:

				//stays in func wait_click until click
				double_click = wait_click();
				if(double_click) main_state = RECORD;
				else main_state = SHAZAM;
				break;


			case RECORD:

				if(song_count == MAX_MEM_SONG) led_animation(ERROR2_LED); //error: too many songs
				else {
					audio(RECORD, song_count);
					led_animation(SUCCESS2_LED);

					if(song_count == 0) signal_rec_traj_sem(); //record color and size of the object

					save_trajectory(song_count);
					led_animation(SUCCESS2_LED);

					song_count ++;
				}

				main_state = WAIT;
				break;

			case SHAZAM:

				if(song_count == 0) led_animation(ERROR2_LED); //error: zero songs memorized
				else{
					current_song = audio(SHAZAM, song_count);
					chThdSleepMilliseconds(1000);

					if(current_song == -1) led_animation(ERROR1_LED); //no matching song
					else {
						led_animation(SUCCESS1_LED);

						set_blinking_state(DANCE_LED);
						dance(current_song);

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
    user_button_start();

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

	//Finite state machine thread
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
