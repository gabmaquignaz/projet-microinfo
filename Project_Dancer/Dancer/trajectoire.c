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
#include <vision.h>

#define PI					3.14159265
#define NB_POS				5
#define WHEEL_DISTANCE      	5.35f    				// [cm]
#define WHEEL_PERIMETER     	13 						// [cm]
#define WHEEL_RADIUS			(13/2*PI)
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)
#define INTERVAL_TEMPS		2 						// [s]
#define INTERVAL_COURT		0.1
#define NSTEP_ONE_TURN		1000
#define OX					0
#define OY					0
#define ORIX					0
#define ORIY					-1


static float pos_pol[2*NB_POS] = {0};
static float pos_car_x[NB_POS] = {0};
static float pos_car_y[NB_POS] = {0};
static float cx[NB_POS], bx[NB_POS-1], dx[NB_POS-1];
static float cy[NB_POS], by[NB_POS-1], dy[NB_POS-1];

static THD_WORKING_AREA(waTrajectoire, 256);
static THD_FUNCTION(Trajectoire, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	for(uint16_t i = 0; i< NB_POS; i++){

		//waits until an position has been captured
		chBSemWait(&dist_ready_sem);

		pos_car_x[i] = get_hor_dist_mm();
		pos_car_y[i] = sqrt(get_real_dist_mm()*get_real_dist_mm()-pos_car_x[i]*pos_car_x[i]);


	}

	convert_pos();
}




float angle_from_three_points(float x1, float y1, float x2, float y2, float x3, float y3){

	float v1x = x2-x1;
	float v1y = y2-y1;
	float v2x = x3-x2;
	float v2y = y3-y2;


	float angle = atan2(v2y, v2x)-atan2(v1y, v1x);
	if (angle>PI) angle -= 2*PI;
	if (angle<-PI) angle += 2*PI;

	return (angle);
}

//not finished, no motor control yet
void convert_pos_step(void){

	//provisoire, entrée des données
	//p0
	pos_pol[0] = 10;
	pos_pol[1] = 10;
	//p1
	pos_pol[2] = 0;
	pos_pol[3] = 10;
	//p2
	pos_pol[4] = -10;
	pos_pol[5] = 10;
	//p3
	pos_pol[6] = -10;
	pos_pol[7] = 0;
	//p4
	pos_pol[8] = 0;
	pos_pol[9] = 0;

	//convert from cartesian (x,y) to (distance, angle)

	float x_mem;

	for(uint8_t i = NB_POS-1; i>1; i--){

		x_mem = pos_pol[2*i];

		//distance between two consecutive points
		pos_pol[2*i] = sqrt((pos_pol[2*i]-pos_pol[2*(i-1)])*(pos_pol[2*i]-pos_pol[2*(i-1)])
							+(pos_pol[2*i+1]-pos_pol[2*(i-1)+1])*(pos_pol[2*i+1]-pos_pol[2*(i-1)+1]));

		//angle between two consecutive vectors
		pos_pol[2*i+1] = angle_from_three_points(pos_pol[2*(i-2)], pos_pol[2*(i-2)+1],
												pos_pol[2*(i-1)], pos_pol[2*(i-1)+1],
												x_mem, pos_pol[2*i+1]);

	}
	//first two sections from origin
	x_mem = pos_pol[2];
	pos_pol[2] = sqrt(pos_pol[2]*pos_pol[2]+pos_pol[3]*pos_pol[3]);
	pos_pol[3] = angle_from_three_points(OX, OY, pos_pol[0], pos_pol[1], x_mem, pos_pol[3]);

	x_mem = pos_pol[0];
	pos_pol[0] = sqrt(pos_pol[0]*pos_pol[0]+pos_pol[1]*pos_pol[1]);
	pos_pol[1] = angle_from_three_points(ORIX, ORIY, OX, OY, x_mem, pos_pol[1]);


	for(uint8_t i = 0; i < NB_POS; i++){
		//Conversion from cm and ° to step
		pos_pol[2*i] *= NSTEP_ONE_TURN/(WHEEL_PERIMETER);
		pos_pol[2*i+1] *= WHEEL_DISTANCE*NSTEP_ONE_TURN/(2*WHEEL_PERIMETER);
	}


	//drive
	for(uint8_t i = 0 ; i < NB_POS ; i++){

		//be sure that the motors are initialized


	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);

}

void convert_pos(void){

	//provisoire, entrée des données
	//p0
	pos_pol[0] = 10;
	pos_pol[1] = 10;
	//p1
	pos_pol[2] = 0;
	pos_pol[3] = 10;
	//p2
	pos_pol[4] = -10;
	pos_pol[5] = 10;
	//p3
	pos_pol[6] = -10;
	pos_pol[7] = 0;
	//p4
	pos_pol[8] = 0;
	pos_pol[9] = 0;

	//convert from cartesian (x,y) to (distance, angle)

	float x_mem;

	for(uint8_t i = NB_POS-1; i>1; i--){

		x_mem = pos_pol[2*i];

		//distance between two consecutive points
		pos_pol[2*i] = sqrt((pos_pol[2*i]-pos_pol[2*(i-1)])*(pos_pol[2*i]-pos_pol[2*(i-1)])
							+(pos_pol[2*i+1]-pos_pol[2*(i-1)+1])*(pos_pol[2*i+1]-pos_pol[2*(i-1)+1]));

		//angle between two consecutive vectors
		pos_pol[2*i+1] = angle_from_three_points(pos_pol[2*(i-2)], pos_pol[2*(i-2)+1],
												pos_pol[2*(i-1)], pos_pol[2*(i-1)+1],
												x_mem, pos_pol[2*i+1]);

	}
	//first two sections from origin
	x_mem = pos_pol[2];
	pos_pol[2] = sqrt(pos_pol[2]*pos_pol[2]+pos_pol[3]*pos_pol[3]);
	pos_pol[3] = angle_from_three_points(OX, OY, pos_pol[0], pos_pol[1], x_mem, pos_pol[3]);

	x_mem = pos_pol[0];
	pos_pol[0] = sqrt(pos_pol[0]*pos_pol[0]+pos_pol[1]*pos_pol[1]);
	pos_pol[1] = angle_from_three_points(ORIX, ORIY, OX, OY, x_mem, pos_pol[1]);


	//***Ajouté par Gab***
	for(uint8_t i = 0; i < NB_POS; i++){
		//Conversion from cm and ° to step/s
		pos_pol[2*i] *= NSTEP_ONE_TURN/(WHEEL_PERIMETER*INTERVAL_TEMPS);
		pos_pol[2*i+1] *= WHEEL_DISTANCE*NSTEP_ONE_TURN/(2*WHEEL_PERIMETER*INTERVAL_TEMPS);
	}


	//drive
	for(uint8_t i = 0 ; i < NB_POS ; i++){

		//be sure that the motors are initialized

		if (abs(pos_pol[2*i+1])>0){
			//rotation
			left_motor_set_speed(-pos_pol[2*i+1]);
			right_motor_set_speed(pos_pol[2*i+1]);
			chThdSleepMilliseconds(1000*INTERVAL_TEMPS);
		}

		//forward
		left_motor_set_speed(pos_pol[2*i]);
		right_motor_set_speed(pos_pol[2*i]);
		chThdSleepMilliseconds(1000*INTERVAL_TEMPS);
	}

	left_motor_set_speed(0);
	right_motor_set_speed(0);

}


//************** NOT USED FOR THE MOMENT *****************
void interpolate(void){

	/*
	float pos_x = 0, pos_y = 0, pos_theta = PI/2;
	float step_l = 0, step_r = 0;
	float step_l_mem = 0, step_r_mem = 0;
	float speed_l = 0, speed_r = 0;
	float speed_x = 0, speed_y = 0, speed_theta = 0;
	float rot_speed = 0, trans_speed = 0;
	float theta_goal = 0;
	*/


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

			/*
			 *
			//dead reckoning
			step_l_mem = step_l;
			step_r_mem = step_r;
			step_l = (left_motor_get_pos()/NSTEP_ONE_TURN)*2*PI;
			step_r = (right_motor_get_pos()/NSTEP_ONE_TURN)*2*PI;

			pos_x = pos_x + cos(pos_theta)*(WHEEL_RADIUS/2)*(step_l + step_r - step_l_mem - step_r_mem);
			pos_y = pos_y + sin(pos_theta)*(WHEEL_RADIUS/2)*(step_l + step_r - step_l_mem - step_r_mem);
			pos_theta = pos_theta + (WHEEL_RADIUS/WHEEL_DISTANCE)*(-step_l + step_r + step_l_mem - step_r_mem);

			speed_x = (WHEEL_RADIUS/2)*(speed_r + speed_l)*cos(pos_theta);
			speed_y = (WHEEL_RADIUS/2)*(speed_r + speed_l)*sin(pos_theta);
			speed_theta = (WHEEL_RADIUS/WHEEL_DISTANCE)*(speed_r - speed_l);

			trans_speed = (WHEEL_RADIUS/2)*(speed_r + speed_l); 				//we want it to be cst
			rot_speed = (WHEEL_RADIUS/WHEEL_DISTANCE)*(speed_r - speed_l);


			//P controller
			theta_goal = atan2()


			speed_r = (2*trans_speed + rot_speed*WHEEL_DISTANCE)/(2*WHEEL_RADIUS);
			speed_l = (2*trans_speed - rot_speed*WHEEL_DISTANCE)/(2*WHEEL_RADIUS);

			//ax[i]+bx[i]*xnew+cx[i]*xnew**2+dx[i]*xnew**3

			*/

		}
	}
}

void trajectoire_start(void){
	chThdCreateStatic(waTrajectoire, sizeof(waTrajectoire), HIGHPRIO, Trajectoire, NULL);
}




