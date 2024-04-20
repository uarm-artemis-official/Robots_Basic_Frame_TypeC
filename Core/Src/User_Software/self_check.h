/*******************************************************************************
* @file           : any_header.h
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

#ifndef __DIRECTORY_ANY_HEADER_H__
#define __DIRECTORY_ANY_HEADER_H__

#include "main.h"
#include "buzzer.h"


/* define general declarations for gimbal task here */
#define SOME_YOUR_VALUE (0.0f)
//#define SILENT_SELF_CHECK

/* define user structure here */
typedef enum{
	CHECK_OK = 0,
	CHECK_FAIL = 1
}SelfCheckStatus_t;

/* extern global variables here */
extern Buzzer_t buzzer;

/* global variable definitions */

/* functions declaration here */
SelfCheckStatus_t self_check_system();
SelfCheckStatus_t self_check_motors(uint8_t motor_id);

#endif /*__DIRECTORY_ANY_HEADER_H__*/
