/*
 * vision.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz
 */

#ifndef VISION_H_
#define VISION_H_

static BSEMAPHORE_DECL(rec_traj_ready_sem, TRUE);

uint16_t get_real_dist_mm(void);
uint16_t get_hor_dist_mm(void);
void process_image_start(void);
void signal_rec_traj_sem(void);

#endif /* VISION_H_ */
