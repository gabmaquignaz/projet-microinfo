/*
 * trajectoire.c
 *
 *  Created on: 6 Apr 2020
 *      Author: maximepoffet
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <trajectoire.h>

#define NB_POS		100

void convert_pos(void){

	float pos_pol[NB_POS];

	float r_one = pos_pol[0];
	float r_two;


	for(uint8_t i = 1 ; i < NB_POS ; i++){

		r_two = r_one;
		r_one = pos_pol[2*i];

		pos_pol[2*i] = sqrt((r_two*r_two) + (r_one*r_one) + 2*r_two*r_one*cos(pos_pol[2*i+1-2]-pos_pol[2*i+1]));

	}

}





