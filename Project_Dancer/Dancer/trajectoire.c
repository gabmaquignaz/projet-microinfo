/*
 * trajectoire.c
 *
 *  Created on: 6 Apr 2020
 *  		Author: maximepoffet
 */

#include "ch.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <trajectoire.h>
#include <motors.h>
#include <vision.h>

#include "chprintf.h"
#include "usbcfg.h"

#define PI					3.14159265
#define NB_POS				100
#define WHEEL_DISTANCE      	53.5f    				// [mm]
#define WHEEL_PERIMETER     	130 						// [mm]
#define WHEEL_RADIUS			(13/2*PI)
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define INTERVAL_TEMPS		0.2 						// [s]
#define INTERVAL_COURT		0.1
#define ROTATION_SPEED		500
#define NSTEP_ONE_TURN		1000
#define OX					0
#define OY					0
#define ORIX					0
#define ORIY					-1
#define MIN_DIST				5
#define NUM_MEM_TRAJ			3 //number of memorized trajectories


static float positions[NUM_MEM_TRAJ][2*NB_POS] = {0};


void save_trajectory(uint8_t traj_count){

	if (traj_count >= NUM_MEM_TRAJ) return;

	for(uint16_t i = 0; i< NB_POS; i++){

		//waits until an position has been captured
		chBSemWait(&dist_ready_sem);
		chprintf((BaseSequentialStream *) &SD3,"position saved\n");


		positions[traj_count][2*i] = get_hor_dist_mm();
		positions[traj_count][2*i+1] = sqrt(get_real_dist_mm()*get_real_dist_mm()
										  -positions[traj_count][2*i]*positions[traj_count][2*i]);

		//filter all positions too close from the last one
		if (sqrt(pow((positions[traj_count][2*(i-1)]-positions[traj_count][2*i]), 2)
				+ pow((positions[traj_count][2*(i-1)+1]-positions[traj_count][2*i+1]), 2)) < MIN_DIST){
			i--;
		}


	}
	chThdSleepMilliseconds(5000);
	chprintf((BaseSequentialStream *) &SD3,"done\n");
	chThdSleepMilliseconds(2000);
	convert_pos(traj_count);

}


void signal_dist_ready_sem(void){
	chBSemSignal(&dist_ready_sem);
}

float vect_to_angle(float vx, float vy){
	if(!vx){
		if(vy > 0) return PI/2;
		else return -PI/2;
	}
	else return atan2(vy, vx);
}

float angle_from_three_points(float x1, float y1, float x2, float y2, float x3, float y3){

	float v1x = x2-x1;
	float v1y = y2-y1;
	float v2x = x3-x2;
	float v2y = y3-y2;

	float angle1 = vect_to_angle(v1x, v1y);
	float angle2 = vect_to_angle(v2x, v2y);

	float angle = angle2 - angle1;
	if (angle>PI) angle -= 2*PI;
	if (angle<-PI) angle += 2*PI;
	return (angle);
}

void dance(uint8_t traj_count){
	chprintf((BaseSequentialStream *) &SD3,"DANCING !\n");

	//drive
	for(uint8_t i = 0 ; i < NB_POS ; i++){

		//rotation
		if (abs(positions[traj_count][2*i+1])>0){
			//left_motor_set_speed(-positions[2*i+1]);
			//right_motor_set_speed(positions[2*i+1]);
			//chThdSleepMilliseconds(1000*INTERVAL_TEMPS);

			left_motor_set_pos(0);
			right_motor_set_pos(0);

			if (positions[traj_count][2*i+1]>0){
				//counter-clockwise rotation
				left_motor_set_speed(-ROTATION_SPEED);
				right_motor_set_speed(ROTATION_SPEED);
			}
			else{
				//clockwise rotation
				left_motor_set_speed(ROTATION_SPEED);
				right_motor_set_speed(-ROTATION_SPEED);
			}

			while (abs(left_motor_get_pos()) < abs(positions[traj_count][2*i+1])
				   || abs(right_motor_get_pos()) < abs(positions[traj_count][2*i+1]));
		}

		//forward
		left_motor_set_pos(0);
		right_motor_set_pos(0);

		left_motor_set_speed(ROTATION_SPEED);
		right_motor_set_speed(ROTATION_SPEED);

		while (left_motor_get_pos() < positions[traj_count][2*i] || right_motor_get_pos() < positions[traj_count][2*i]);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);

}

void convert_pos(uint8_t traj_count){

	float x_mem;

	//first conversion from cartesian (x,y) to (distance, angle)

	for(uint8_t i = NB_POS-1; i>1; i--){

		x_mem = positions[traj_count][2*i];

		//distance between two consecutive points
		positions[traj_count][2*i] = sqrt((positions[traj_count][2*i]-positions[traj_count][2*(i-1)])
										*(positions[traj_count][2*i]-positions[traj_count][2*(i-1)])
										+(positions[traj_count][2*i+1]-positions[traj_count][2*(i-1)+1])
										*(positions[traj_count][2*i+1]-positions[traj_count][2*(i-1)+1]));

		//angle between two consecutive vectors
		positions[traj_count][2*i+1] = angle_from_three_points(positions[traj_count][2*(i-2)], positions[traj_count][2*(i-2)+1],
															 positions[traj_count][2*(i-1)], positions[traj_count][2*(i-1)+1],
															 x_mem, positions[traj_count][2*i+1]);

	}
	//first two sections from origin
	x_mem = positions[traj_count][2];
	positions[traj_count][2] = sqrt((positions[traj_count][2]-positions[traj_count][0])
								  *(positions[traj_count][2]-positions[traj_count][0])
								  +(positions[traj_count][3]-positions[traj_count][1])
								  *(positions[traj_count][3]-positions[traj_count][1]));

	positions[traj_count][3] = angle_from_three_points(OX, OY,
													 positions[traj_count][0], positions[traj_count][1],
													 x_mem, positions[traj_count][3]);

	x_mem = positions[traj_count][0];
	positions[traj_count][0] = sqrt(positions[traj_count][0]*positions[traj_count][0]
								  +positions[traj_count][1]*positions[traj_count][1]);

	positions[traj_count][1] = angle_from_three_points(ORIX, ORIY, OX, OY, x_mem, positions[traj_count][1]);


	//second conversion

	for(uint8_t i = 0; i < NB_POS; i++){
		//Conversion from [mm] to [steps]
		positions[traj_count][2*i] *= NSTEP_ONE_TURN/(WHEEL_PERIMETER);
		//positions[2*i+1] *= WHEEL_DISTANCE*NSTEP_ONE_TURN/(2*WHEEL_PERIMETER*INTERVAL_TEMPS);

		//Conversion from [radians] to [steps]
		positions[traj_count][2*i+1] *=WHEEL_DISTANCE*NSTEP_ONE_TURN/(2*WHEEL_PERIMETER);

	}
}

