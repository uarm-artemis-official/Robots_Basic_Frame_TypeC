/*******************************************************************************
* @file           : IMU_App.c
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : James
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __IMU_APP_C__
#define __IMU_APP_C__

#include "apps_defines.h"
#include "IMU_App.h"

#include "buzzer.h"
#include "imu.h"
#include "public_defines.h"
#include "message_center.h"
#include "event_center.h"
#include "debug.h"
#include "pid.h"


static IMU_t imu;
static IMU_Heat_t imu_heating_control;

/**
  * @brief     IMU task main entry function
  * @retval    None
  */
void IMU_Task_Function(void const * argument) {

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_EXEC_TIME);

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	Attitude_t attitude;
	float message_data[2];

	imu_task_init(&imu, &imu_heating_control);
	/* main imu task begins */
	for(;;){
		imu_temp_pid_control(&imu_heating_control);
		/* read the mpu data */

		if (imu.temp_status == NORMAL) {
			get_attitude(&attitude);

			message_data[0] = attitude.yaw;
			message_data[1] = attitude.pitch;
			pub_message(IMU_READINGS, message_data);
		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


void calibrate_imu(IMU_t *imu, IMU_Heat_t *heat_control) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(imu->temp_status != NORMAL){
		imu->temp = get_imu_temp();
		imu_temp_pid_control(heat_control);
		vTaskDelayUntil(&xLastWakeTime, IMU_CALI_FREQ);
	}
	set_imu_offset();
}


void imu_task_init(IMU_t *imu, IMU_Heat_t *heat_control) {
	memset(imu, 0, sizeof(IMU_t));
	memset(heat_control, 0, sizeof(IMU_Heat_t));

	init_imu();

	/* init sensor pid */
//	pid_param_init(&(imu.tmp_pid), 4000, 1500, 25, 1000, 0.1, 1000);
	pid2_init(&(heat_control->pid), 1200, 220, 0, 1, 1, 0, 4000);
	set_imu_temp_status(&imu, ABNORMAL);
	imu->imu_mode = GA_MODE; // forbid ist8310
    if(imu->imu_mode == GA_MODE){
    	// no use ist8310
		imu->ahrs_sensor.mx = 0.0f;
		imu->ahrs_sensor.my = 0.0f;
		imu->ahrs_sensor.mz = 0.0f;
    }
//	imu.sample_time = DWT_Get();
	imu->temp = 0.0;
	calibrate_imu(imu, heat_control);
}

/**
  * @brief     set IMU temp status
  * @param[in] pimu: main imu sturct
  * @param[in] status: IMU_temp_status enum variable
  * @retval    None
  */
void set_imu_temp_status(IMU_t *pimu, IMU_temp_status status){
	pimu->temp_status = status;

	if (pimu->temp_status == NORMAL) {
		emit_events(IMU_READY);
	} else {
		clear_events(IMU_READY);
	}
}


/**
  * @brief  temperature of imu pid control
  * @param[in]: Not used
  * @retval 0
  */
int32_t imu_temp_pid_control(IMU_Heat_t *control) {
	float temp = get_imu_temp();
	float temp_threshold = 0.875f;

	pid2_single_loop_control(&(control->pid), DEFAULT_IMU_TEMP, temp, IMU_TASK_EXEC_TIME * 0.001f * 100); // pid control
	set_imu_pwm(control->pid.total_out);

	if (DEFAULT_IMU_TEMP - temp_threshold <= temp && temp <= DEFAULT_IMU_TEMP + temp_threshold) {
		set_led_state(RED, OFF);
		set_imu_temp_status(&imu, NORMAL);
	} else {
		set_led_state(RED, ON);
		set_imu_temp_status(&imu, ABNORMAL);
	}
	return 0;
}

#endif
