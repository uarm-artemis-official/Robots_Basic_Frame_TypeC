/*******************************************************************************
* @file           : Timer_App.c
* @brief          : A software timer task to register different periodical task.
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef SRC_APPLICATIONS_TIMER_APP_H_
#define SRC_APPLICATIONS_TIMER_APP_H_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "public_defines.h"
#include "message_center.h"
#include "motor.h"



extern CAN_HandleTypeDef hcan1;

void Timer_Task_Func(void const * argument);



#endif /* SRC_APPLICATIONS_TIMER_APP_H_ */
