/*
 * trajectoire.h
 *
 *  Created on: 6 Apr 2020
 *      Author: maximepoffet
 */

#ifndef TRAJECTOIRE_H_
#define TRAJECTOIRE_H_

static BSEMAPHORE_DECL(dist_ready_sem, TRUE);

void convert_pos(uint8_t traj_count);
void signal_dist_ready_sem(void);

#endif /* TRAJECTOIRE_H_ */
