/*******************************************************************************
* @file           : feedforward.h
* @brief          : Use feedforward control to improve accuracy of the response of the object
*                   based on input value
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

#include "main.h"

/* define general declarations for gimbal task here */
#define CHASSIS_FF_GAIN 100 //roughlt setting
#define GIMBAL_FF_GAIN 100
#define SHOOT_FF_GAIN 100


/* define user structure here */
/**
  * @brief  feedforward main structure
  * @Note
  */
typedef struct{
	float ff_gain;
	float last_input;
	float output;
}FeedForward_t;

/* extern global variables here */


/* define user created variables here */



/* functions declaration here */
void ff_param_init(FeedForward_t *ff, float kf);
float feedforward(FeedForward_t *ff, float input);






#endif /*__FEEDFORWARD_H__*/
