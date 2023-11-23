/*******************************************************************************
* @file           : fuzzy_control.h
* @brief          : use the fuzzy control for dynamically adjust parameters
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __FUZZY_CONTROL_H__
#define __FUZZY_CONTROL_H__

#include "pid.h"

/* define general declarations for gimbal task here */


/* define user structure here */
/**
  * @brief     briefly introduce your structure
  */


/* extern global variables here */


/* define user created variables here */



/* functions declaration here */
void fuzzy_tune_pid(PID_t *pid, float error);






#endif /*__FUZZY_CONTROL_H__*/
