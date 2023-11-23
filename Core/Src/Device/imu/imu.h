/*******************************************************************************
* @file           : imu.h
* @brief          : gyroscope BMI088 read/write for attitude reading
* @restructed     : Nov, 2023
* @maintainer     : Haoran
********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __IMU_H__
#define __IMU_H__

/* define general declarations here */
#define IMU_OK  0
#define IMU_ERR 1
#define DEFAULT_IMU_TEMP 45

#ifdef DRV_IMU_H_GLOBAL
    #define DRV_IMU_H_EXTERN
#else
    #define DRV_IMU_H_EXTERN extern
#endif

#include "stm32f4xx_hal.h"
#include "ahrs.h"
#include "pid.h"
#include "bmi088_driver.h"

#define BMI088_PARAM_KEY "BMI088_PARAM"
/**
  * @brief imu main structure
  * @Note
  */
typedef enum{
	NORMAL = 0,
	ABNORMAL
}IMU_temp_status;

typedef enum{
	GAM_MODE = 0, // 9 axis imu
	GA_MODE 	  // 6 axis mpu
}IMU_mode_t;

typedef struct{
	float temp;
	uint32_t sample_time;
	PID_t tmp_pid;

	IMU_temp_status temp_status;
	IMU_mode_t imu_mode;
	AhrsSensor_t ahrs_sensor;//for ahrs sensor - processed data
}IMU_t;
IMU_t imu;

extern bmi088_real_data_t bmi088_real_data;

void bmi088_get_data(AhrsSensor_t *sensor);
void bmi088_get_temp(float *tmp);
uint8_t bmi088_device_init(void);
int ahrs_update(AhrsSensor_t *sensor, uint8_t period_ms);
int32_t imu_temp_keep(void *argc);
void imu_temp_ctrl_init(void);
uint8_t bmi088_set_offset(void);
uint8_t bmi088_get_offset(void);

#endif
