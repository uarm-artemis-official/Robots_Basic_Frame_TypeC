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

#include "apps_types.h"


void IMU_Task_Function(void const * argument);
void imu_task_init(IMU_t *imu, IMU_Heat_t *heat_control);
void calibrate_imu(IMU_t *imu, IMU_Heat_t *heat_control);
int32_t imu_temp_pid_control(IMU_Heat_t *control);

void set_imu_temp_status(IMU_t *pimu, IMU_temp_status status);


#endif /*__SRC_IMU_APP_H__*/