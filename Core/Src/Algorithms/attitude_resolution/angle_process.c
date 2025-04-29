/*******************************************************************************
* @file           : angle_process.c
* @brief          : use kalman filter to parse the angle from raw ampu data
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __ANGLE_PROCESS_C__
#define __ANGLE_PROCESS_C__

#include "angle_process.h"
#include "maths.h"
#include "arm_math.h"

/* define internal vars */
//static AhrsSensor_t temp_ahrs;
/* define internal functions */
/***********************************************************************************
 * establish the state space rep(tp_bias = temperature bias)
 * state:
 * 		[ angle   ] = [ 1  -dt       * [ angle        [ W
 * 		[ tp_bias ]     0   1 ] (A)      tp_bias ]  +
 *
 * obs
 * 		[ angle ] = [ 1  0 ](H)  * [ angle
 * 		[     0 ] 	      		     tp_bias ]
 *
 * ************************************************************************************/
void kf_atti_calc(KalmanFilter_t *kf)
{
	/* Not Impelement */
}

/**
  * @brief     use kalman filter to estimate the optimal
  * @param[in] imu handler
  * @param[in] 2nd param intro
  * @retval    return values intro
  */
void atti_math_calc(AhrsSensor_t *ahrs, Attitude_t *atti){

	float accel_pitch = 0;
	float accel_roll  = 0;

	// accel a -> m^2/s
	// gyro  w -> rad/s

	/* apply low pass filter for accel(accel is easier to be affected by high-freq noise) */

	/* apply complementary filter to calc latest angle */
	/* calc the tilt angles (Pitch and Roll) obtained from the accelerometer*/
	accel_pitch = atan2f(ahrs->ay, 1.0f / invSqrt(ahrs->ax * ahrs->ax + ahrs->az * ahrs->az));
	accel_roll = atan2f(-ahrs->ax, 1.0f / invSqrt(ahrs->ay * ahrs->ay + ahrs->az * ahrs->az));

	/* update the estimated value of the tilt angle */
	// note ahrs->wx * dt is acatually the integral of the wx (rate)
	atti->pitch = GYRO_WEIGHT * (atti->pitch + ahrs->wx * SAMPLE_DT) + ACCEL_WEIGHT * accel_pitch; //accel cali
	atti->roll  = GYRO_WEIGHT * (atti->roll + ahrs->wy * SAMPLE_DT)   + ACCEL_WEIGHT * accel_roll;  //accel cali
	atti->yaw   += ahrs->wz * SAMPLE_DT; // no accel cali

	/* for debugging */
	atti->pitch = atti->pitch * RAD2DEGEE;
	atti->roll  = atti->roll * RAD2DEGEE;
	atti->yaw   = atti->yaw * RAD2DEGEE;
}



#endif /*__ANGLE_PROCESS_C__*/
