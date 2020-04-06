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

#define NB_POS		100
#define ECART		5
#define RAYON		2
#define PI			3.1415


void convert_pos(void){

	float pos_pol[2*NB_POS] = {0};

	//essais qui marche mais moche
	/*
	float r_one = pos_pol[0];
	float r_two;


	for(uint8_t i = 1 ; i < NB_POS ; i++){

		r_two = r_one;
		r_one = pos_pol[2*i];

		pos_pol[2*i] = sqrt((r_two*r_two) + (r_one*r_one) + 2*r_two*r_one*cos(pos_pol[2*i+1-2]-pos_pol[2*i+1]));

		pos_pol[2*i+1] =

	}
	*/

	for(uint8_t i = NB_POS ; i > 1 ; i--){

		float r_mem = pos_pol[2*i];

		//distance between two consecutive points
		pos_pol[2*i] = sqrt((pos_pol[2*i-2]*pos_pol[2*i-2])
							+ (pos_pol[2*i]*pos_pol[2*i])
							+ 2*pos_pol[2*i-2]*pos_pol[2*i]*cos(pos_pol[2*i+1-2]-pos_pol[2*i+1]));

		//angle between two consecutive vectors
		pos_pol[2*i+1] = PI - (r_mem*sin(pos_pol[2*i+1-2] - pos_pol[2*i+1]))/pos_pol[2*i];
	}



	/*
	for(uint8_t i = 0 ; i < NB_POS ; i++){



	}
	*/

}





