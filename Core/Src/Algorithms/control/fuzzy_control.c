/*******************************************************************************
* @file           : fuzzy_control.c
* @brief          : use the fuzzy control for dynamically adjust parameters
* @created time	  : Aug, 2023
* @author         : Haoarn
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __FUZZY_CONTROL_C__
#define __FUZZY_CONTROL_C__

#include "fuzzy_control.h"

/* @attention
 * 		This file is created for dynamically adjust parameters during the competition, especially for
 * 	yaw and pitch motor. If we use upper loading system, the actual payload handled by the gimbal motors
 * 	will change as the operator launching the projectiles. This fuzzy control would suppress this strong
 * 	tendency to change results in a more stable gimbal, thus enhancing the overall robustness of the robots.
 *
 * More infomation:
 * 	Paper: Adaptive Fuzzy PID Controller for A Compact Autonomous Underwater Vehicle
 * 			https://ieeexplore.ieee.org/document/9389483
 * 	Paper: Deployment of a Low Cost Fuzzy Controller Using Open Source Embedded Hardware and
            Software Tools
            https://www.iaeng.org/publication/IMECS2018/IMECS2018_pp653-658.pdf
 *
 * */


/* define internal vars */

/* define internal functions */


/**
  * @brief     Fuzzy control pid paramters (demo)
  * @param[in] pid main struct
  * @param[in] error value
  * @retval    None
  */
void fuzzy_tune_pid(PID_t *pid, float error) {
    if (fabs(error) < 0.1) {
        // If error is very small
        pid->kp += 0.01;  // Small increment in kp
        pid->ki += 0.001;  // Small increment in ki
        pid->kd += 0.05;  // Small increment in kd
    }
    else if (fabs(error) < 1) {
        // If error is moderate
        pid->kp += 0.05;
        pid->ki += 0.01;
        pid->kd += 0.1;
    }
    else {
        // If error is large
        pid->kp += 0.1;
        pid->ki += 0.05;
        pid->kd += 0.2;
    }
}


#endif /*__FUZZY_CONTROL_C__*/
