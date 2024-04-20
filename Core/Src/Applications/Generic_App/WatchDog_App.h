/*******************************************************************************
* @file           : WatchDog_App.h
* @brief          : A watchdog program monitoring the system status in real time
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __WATCHDOG_APP_H__
#define __WATCHDOG_APP_H__

#include "self_check.h"
#include "buzzer.h"
//#include "stm32f4xx_hal_iwdg.h


extern IWDG_HandleTypeDef hiwdg;

void wdg_task_init(void);
void WatchDog_Task_Function(void);
void wdg_daemon_feed_dog(void);

#endif /* __WATCHDOG_APP_H__ */
