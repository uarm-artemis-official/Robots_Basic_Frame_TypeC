/*******************************************************************************
* @file           : IMU_App.h
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SRC_IMU_APP_H__
#define __SRC_IMU_APP_H__

#include "Gimbal_App.h"


/* define general declarations for gimbal task here */
#define DEFAULT_IMU_TEMP 40 		// degrees centigrade
#define IMU_TMP_PWM_HTIM    htim10
#define IMU_TMP_PWM_CHANNEL  TIM_CHANNEL_1

/* define user structure here */


/* extern global variables here */
extern TIM_HandleTypeDef htim3;
extern IMU_t imu;
extern uint8_t imu_init_flag;

/* define user created variables here */



/* functions declaration here */
void IMU_Task_Function(void);
void set_imu_temp_status(IMU_t *pimu, IMU_temp_status status);
int32_t imu_temp_pid_control(void);
void imu_task_init(void);















#endif /*__SRC_IMU_APP_H__*/
