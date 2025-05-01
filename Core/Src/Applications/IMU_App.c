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

#include "apps_config.h"
#include "IMU_App.h"


static IMU_t imu;

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

	imu_task_init(xFrequency);
	/* main imu task begins */
	for(;;){
		/* set watch point */
		/* IMU temperature PID control*/
		bmi088_get_data(&imu.ahrs_sensor, &(imu.temp));
		imu_temp_pid_control();
		/* read the mpu data */

		if (imu.temp_status == NORMAL) {
			madgwick_ahrs_update(&imu.ahrs_sensor, &attitude);

			message_data[0] = attitude.yaw;
			message_data[1] = attitude.pitch;
			pub_message(IMU_READINGS, message_data);

		}

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


static void calibrate_imu(TickType_t xFrequency) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(imu.temp_status != NORMAL){
		imu.temp = get_BMI088_temperature();
		imu_temp_pid_control();
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

	/* set the offset when the temperature reach normal status */
	bmi088_get_offset();

	/* imu init finished */
//	buzzer_play_mario(300);
}


void imu_task_init(TickType_t xFrequency) {
	/* inint bmi088 */
	bmi088_device_init();
	ist8310_init();
	/* init sensor pid */
//	pid_param_init(&(imu.tmp_pid), 4000, 1500, 25, 1000, 0.1, 1000);
	pid2_init(&(imu.tmp_pid), 1200, 220, 0, 1, 1, 0, 4000);
	set_imu_temp_status(&imu, ABNORMAL);
	imu.imu_mode = GA_MODE; // forbid ist8310
    if(imu.imu_mode == GA_MODE){
    	// no use ist8310
		imu.ahrs_sensor.mx = 0.0f;
		imu.ahrs_sensor.my = 0.0f;
		imu.ahrs_sensor.mz = 0.0f;
    }
//	imu.sample_time = DWT_Get();
	imu.temp = 0.0;
	calibrate_imu(xFrequency);
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

void set_imu_pwm(IMU_t *pimu, uint16_t pwm){
	if (pwm < 0) pwm = 0;
	__HAL_TIM_SET_COMPARE(&IMU_TMP_PWM_HTIM, IMU_TMP_PWM_CHANNEL, pwm);
}

/**
  * @brief  temperature of imu pid control
  * @param[in]: Not used
  * @retval 0
  */
static int32_t counter = 0; // TODO: remove
int32_t imu_temp_pid_control(void) {
	float temp = imu.temp;
	float temp_threshold = 0.875f;

	counter = (counter + 1) % 100;
	if (counter == 0 && temp < 50) {
		pid2_single_loop_control(&(imu.tmp_pid), DEFAULT_IMU_TEMP, temp, IMU_TASK_EXEC_TIME * 0.001f * 100); // pid control
		set_imu_pwm(&imu, imu.tmp_pid.total_out);
	}
	if (DEFAULT_IMU_TEMP - temp_threshold <= temp && temp <= DEFAULT_IMU_TEMP + temp_threshold) {
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
		set_imu_temp_status(&imu, NORMAL);
	} else {
		HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
		set_imu_temp_status(&imu, ABNORMAL);
	}
	return 0;
}

#endif
