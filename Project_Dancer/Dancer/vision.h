/*
 * vision.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#ifndef VISION_H_
#define VISION_H_

uint16_t get_real_dist_mm(void);
uint16_t get_hor_dist_mm(void);
void process_image_start(void);

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640

#endif /* VISION_H_ */
