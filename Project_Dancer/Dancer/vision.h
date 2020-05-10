/*
 * vision.h
 *
 *  Created on: 2 Apr 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#ifndef VISION_H_
#define VISION_H_

static BSEMAPHORE_DECL(rec_traj_ready_sem, TRUE);

float get_real_dist_mm(void);
float get_hor_dist_mm(void);
void process_image_start(void);
void signal_rec_traj_sem(void);
void vision_set_active(bool val);

#endif /* VISION_H_ */
