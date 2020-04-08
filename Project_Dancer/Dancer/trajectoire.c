/*
 * trajectoire.c
 *
 *  Created on: 6 Apr 2020
 *      Author: maximepoffet
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <trajectoire.h>
#include <motors.h>

#define NB_POS				5
#define WHEEL_DISTANCE      	5.35f    				// [cm]
#define WHEEL_PERIMETER     	13 						// [cm]
#define PI					3.14159265
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define INTERVAL_TEMPS		1 						// [s]
#define INTERVAL_COURT		0.1
#define NSTEP_ONE_TURN		4000


static float pos_pol[2*NB_POS] = {0};
static float pos_car_x[NB_POS] = {0};
static float pos_car_y[NB_POS] = {0};
static float cx[NB_POS], bx[NB_POS-1], dx[NB_POS-1];
static float cy[NB_POS], by[NB_POS-1], dy[NB_POS-1];

void convert_pos(void){

	//provisoire, entrée des données
	//p0
	pos_pol[0] = 1.414;
	pos_pol[1] = 0.785;
	//p1
	pos_pol[2] = 1;
	pos_pol[3] = 1.57;
	//p2
	pos_pol[4] = 1.414;
	pos_pol[5] = 2.356;
	//p3
	pos_pol[6] = 1;
	pos_pol[7] = 3.141;
	//p4
	pos_pol[8] = 0;
	pos_pol[8] = 0;

	float r_mem;

	for(uint8_t i = NB_POS-1; i>0; i--){

		//pour le premier point (i=0), l'angle et le rayon reste le même

		r_mem = pos_pol[2*i];
		//distance between two consecutive points
		pos_pol[2*i] = sqrt((pos_pol[2*i-2]*pos_pol[2*i-2])
							+ (pos_pol[2*i]*pos_pol[2*i])
							+ 2*pos_pol[2*i-2]*pos_pol[2*i]*cos(pos_pol[2*i+1-2]-pos_pol[2*i+1]));

		//angle between two consecutive vectors
		pos_pol[2*i+1] = PI - (r_mem*sin(pos_pol[2*i+1-2] - pos_pol[2*i+1]))/pos_pol[2*i];
	}

	//convert into speeds [steps/s]
	for(int16_t i = NB_POS-1 ; i >= 0 ; i--){

		pos_pol[2*i] = pos_pol[2*i] / (INTERVAL_TEMPS*WHEEL_PERIMETER);
		pos_pol[2*i+1] = (pos_pol[2*i+1]*NSTEP_ONE_TURN*WHEEL_DISTANCE)/(2*INTERVAL_TEMPS*WHEEL_PERIMETER);
	}

	for(uint8_t i = 0 ; i < NB_POS ; i++){

		//be sure that the motors are initialized

		//rotation
		left_motor_set_speed(-pos_pol[2*i+1]);
		right_motor_set_speed(pos_pol[2*i+1]);
		chThdSleepMilliseconds(1000*INTERVAL_TEMPS);

		//forward
		left_motor_set_speed(pos_pol[2*i]);
		right_motor_set_speed(pos_pol[2*i]);
		chThdSleepMilliseconds(1000*INTERVAL_TEMPS);

	}

}


void interpolate(void){

	int n = NB_POS-1, i, j;

	float A[NB_POS-1], l[NB_POS], u[NB_POS], z[NB_POS];

	// ***************** INTERPOLATE X ********************

	for (i = 1; i <= n - 1; ++i){
		A[i] = 3 * (pos_car_x[i + 1] - pos_car_x[i]) - 3 * (pos_car_x[i] - pos_car_x[i - 1]);
	}

	l[0] = 1;
	u[0] = 0;
	z[0] = 0;

	for (i = 1; i <= n - 1; ++i) {
		l[i] = 2 * 2 - u[i - 1];
		u[i] = 1 / l[i];
		z[i] = (A[i] - 1 * z[i - 1]) / l[i];
	}

	l[n] = 1;
	z[n] = 0;
	cx[n] = 0;

	for (j = n - 1; j >= 0; --j) {
	    cx[j] = z[j] - u[j] * cx[j + 1];
	    bx[j] = (pos_car_x[j + 1] - pos_car_x[j]) - (cx[j + 1] + 2 * cx[j]) / 3;
	    dx[j] = (cx[j + 1] - cx[j]) /3;
	}


	// ***************** INTERPOLATE Y ********************

	for (i = 1; i <= n - 1; ++i){
		A[i] = 3 * (pos_car_y[i + 1] - pos_car_y[i]) - 3 * (pos_car_y[i] - pos_car_y[i - 1]);
	}

	l[0] = 1;
	u[0] = 0;
	z[0] = 0;

	for (i = 1; i <= n - 1; ++i) {
		l[i] = 2 * 2 - u[i - 1];
		u[i] = 1 / l[i];
		z[i] = (A[i] - 1 * z[i - 1]) / l[i];
	}

	l[n] = 1;
	z[n] = 0;
	cy[n] = 0;

	for (j = n - 1; j >= 0; --j) {
		cy[j] = z[j] - u[j] * cy[j + 1];
		by[j] = (pos_car_y[j + 1] - pos_car_y[j]) - (cy[j + 1] + 2 * cy[j]) / 3;
		dy[j] = (cy[j + 1] - cy[j]) /3;
	}

	// drive

	for(i=0; i<n-1; i++){
		for(int t = INTERVAL_COURT; t<INTERVAL_TEMPS; t+=INTERVAL_COURT){
			//ax[i]+bx[i]*xnew+cx[i]*xnew**2+dx[i]*xnew**3

		}
	}



}


