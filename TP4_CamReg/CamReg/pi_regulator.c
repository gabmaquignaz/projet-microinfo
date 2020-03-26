#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

#define WANTED_DIST			10
#define WANTED_OR			(640/2)
#define KI					0.001
#define KP					600
#define ARW					500
#define TRESHOLD				0.1


static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    float speed = 0;
    float rot_speed =0;


    float error = 0;
    float or_error = 0;
    static float sum_error = 0;


    while(1){
        time = chVTGetSystemTime();


        //sum_error = error;
        error = get_distance_cm() - WANTED_DIST;
        or_error = get_hor_pos() - WANTED_OR;

        sum_error += error;

        if (sum_error > ARW) sum_error = ARW;
        if (sum_error < -ARW) sum_error = -ARW;


        speed = KP*error + KI*sum_error;
        rot_speed = KP*or_error/1000;

        if (fabs(speed) < TRESHOLD) speed = 0;

        right_motor_set_speed(speed - rot_speed);
        left_motor_set_speed(speed + rot_speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
