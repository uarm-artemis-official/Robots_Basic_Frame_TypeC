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

#ifdef __cplusplus
extern "C" {
#endif

#include "control_types.h"

/* functions declaration here */
void fuzzy_tune_pid(PID_t *pid, float error);

#ifdef __cplusplus
}
#endif

#endif /*__FUZZY_CONTROL_H__*/
