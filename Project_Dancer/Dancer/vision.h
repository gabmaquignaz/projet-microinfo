/*
 * vision.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#ifndef VISION_H_
#define VISION_H_

#define M						3
#define N						5

uint16_t get_real_dist_mm(void);
uint16_t get_hor_dist_mm(void);
void process_image_start(void);
int32_t det_3_3 (float matrix [M][N]);
void do_gauss (float matrix [M][N]);
void exchange_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b);
void substract_lines(float matrix [M][N], uint8_t line_a, uint8_t line_b, float factor);
bool dist_measure (uint8_t* image, uint16_t size);
void background_set(uint16_t *background, uint8_t *image, uint8_t counter);
void background_ignore(uint16_t *background, uint8_t *image);

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640

#endif /* VISION_H_ */
