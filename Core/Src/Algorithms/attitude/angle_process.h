/*******************************************************************************
* @file           : angle_process.h
* @brief          : any header template
* @created time	  : Jul, 2023
* @author         : <Name>
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/*
 * if you copy this file from other OPEN-SOURCE project, please do not
 * add this declaration above. Instead, you are supposed to add their
 * licenses or copyright declaration. Please check ramp.c for more details
 *
 * */

#ifndef __ANGLE_PROCESS_H__
#define __ANGLE_PROCESS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "attitude_types.h"

/* functions declaration here */
void atti_math_calc(AhrsSensor_t *ahrs, Attitude_t *atti);

#ifdef __cplusplus
}
#endif

#endif /*__ANGLE_PROCESS_H__*/
