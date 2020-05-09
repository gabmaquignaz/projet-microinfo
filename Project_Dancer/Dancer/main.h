/*
 * main.h
 *
 *  Created on: 2 May 2020
 *      Author: Gabriel Maquignaz & Maxime P. Poffet
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

enum Main_states {WAIT, RECORD, SHAZAM};

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
