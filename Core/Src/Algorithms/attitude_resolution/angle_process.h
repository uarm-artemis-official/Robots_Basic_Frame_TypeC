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
#include "main.h"
#include "ahrs.h"
#include "kalman_filters.h"


/* define general declarations for gimbal task here */
#define ACCEL_WEIGHT 0.01f  // Define the weight of the accelerometer data, adjust according to the actual situation
#define GYRO_WEIGHT  (1.0f-ACCEL_WEIGHT)  // Define the weight of the gyroscope data, adjust according to the actual situation
#define dt 0.0001f  // Define the sampling period, adjust according to the actual situation

/* define user structure here */


/* extern global variables here */


/* define user created variables here */



/* functions declaration here */
void atti_math_calc(AhrsSensor_t *ahrs, Attitude_t *atti);






#endif /*__ANGLE_PROCESS_H__*/
