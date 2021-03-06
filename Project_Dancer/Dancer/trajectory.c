/*
 * trajectory.c
 *
 *  Created on: 6 Apr 2020
 *  		Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#include <stdlib.h>
#include <math.h>

#include "ch.h"

#include "trajectory.h"
#include "motors.h"
#include "vision.h"
#include "blinking_leds.h"

#define PI					3.14159265
#define NB_POS				75 					//number of positions saved
#define WHEEL_DISTANCE      	53.5f    			// [mm]
#define WHEEL_PERIMETER     	130 					// [mm]
#define WHEEL_RADIUS			(13/2*PI)
#define RADIUS_EPUCK     	35					// [mm]
#define ROTATION_SPEED		500
#define NSTEP_ONE_TURN		1000
#define MIN_DIST				5 					// minimum distance between two consecutive points
#define NUM_MEM_TRAJ			3 					//number of memorized trajectories

static float positions[NUM_MEM_TRAJ][2*NB_POS] = {0};


void save_trajectory(uint8_t traj_count){

	if (traj_count >= NUM_MEM_TRAJ) return;

	for(uint16_t i = 0; i< NB_POS; i++){

		//waits until a position has been captured
		chBSemWait(&dist_ready_sem);

		positions[traj_count][2*i] = get_hor_dist_mm();
		positions[traj_count][2*i+1] = sqrt(get_real_dist_mm()*get_real_dist_mm()
										  -positions[traj_count][2*i]*positions[traj_count][2*i]);

		//filter all positions too close from the last one
		if (sqrt(pow((positions[traj_count][2*(i-1)]-positions[traj_count][2*i]), 2)
				+ pow((positions[traj_count][2*(i-1)+1]-positions[traj_count][2*i+1]), 2)) < MIN_DIST){
			i--;
		}
		else led_animation(REC_TRAJ_LED);

	}
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

	//place the center of the robot on the starting point
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	left_motor_set_speed(ROTATION_SPEED);
	right_motor_set_speed(ROTATION_SPEED);

	while (left_motor_get_pos() < RADIUS_EPUCK
		   || right_motor_get_pos() < RADIUS_EPUCK);

	left_motor_set_speed(0);
	right_motor_set_speed(0);

	//move from point to point
	for(uint8_t i = 2 ; i < NB_POS ; i++){

		//rotation
		if (abs(positions[traj_count][2*i+1])>0){

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

		while (left_motor_get_pos() < positions[traj_count][2*i]
			   || right_motor_get_pos() < positions[traj_count][2*i]);
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

	//second conversion

	for(uint8_t i = 2; i < NB_POS; i++){
		//Conversion from [mm] to [steps]
		positions[traj_count][2*i] *= NSTEP_ONE_TURN/(WHEEL_PERIMETER);

		//Conversion from [radians] to [steps]
		positions[traj_count][2*i+1] *=WHEEL_DISTANCE*NSTEP_ONE_TURN/(2*WHEEL_PERIMETER);

	}
}

