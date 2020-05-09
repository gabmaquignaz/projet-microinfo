/*
 * trajectory.h
 *
 *  Created on: 6 Apr 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

static BSEMAPHORE_DECL(dist_ready_sem, TRUE);

void save_trajectory(uint8_t traj_count);
void convert_pos(uint8_t traj_count);
void dance(uint8_t traj_count);
void signal_dist_ready_sem(void);

#endif /* TRAJECTORY_H_ */
