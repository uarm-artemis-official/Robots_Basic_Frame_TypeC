/*******************************************************************************
* @file           : IMU_App.c
* @brief          : imu temp control and mpu get data
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __IMU_APP_C__
#define __IMU_APP_C__

#include "IMU_App.h"
#include "buzzer.h"
#include "imu.h"

uint8_t imu_init_flag = 0;

/**
  * @brief     IMU task main entry function
  * @retval    None
  */
/* Task execution time (per loop): 1 ms */
void IMU_Task_Function(void){

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // task exec period 1ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	/* main imu task begins */
	for(;;){

		/* set watch point */
		if( imu_init_flag != 1){//gimbal_cali_done_flag == 1 &&

			/* init imu parameters */
			imu_task_init();

			while(imu.temp_status != NORMAL){
				imu.temp = get_BMI088_temperature();
				imu_temp_pid_control();
				vTaskDelayUntil(&xLastWakeTime, xFrequency);
			}

			/* set the offset when the temperature reach normal status */
			__HAL_TIM_SET_COMPARE(&IMU_TMP_PWM_HTIM, IMU_TMP_PWM_CHANNEL, 1000);//small current to keep tmp
			bmi088_get_offset();

			/* imu init finished */
			imu_init_flag = 1;
			buzzer_play_mario(300);
			}

		/* IMU temperature PID control*/
		imu_temp_pid_control();
		/* read the mpu data */
		if(imu_init_flag == 1){
			bmi088_get_data(&imu.ahrs_sensor);
		}

		/* delay utill wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}

void imu_task_init(void){
	/* inint bmi088 */
	bmi088_device_init();
	/* init sensor pid */
	pid_param_init(&(imu.tmp_pid), 2000, 1500, 25, 900, 0.10, 0.05);
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
}

/**
  * @brief     set IMU temp status
  * @param[in] pimu: main imu sturct
  * @param[in] status: IMU_temp_status enum variable
  * @retval    None
  */
void set_imu_temp_status(IMU_t *pimu, IMU_temp_status status){
	pimu->temp_status = status;
}

void set_imu_pwm(IMU_t *pimu, uint16_t pwm){
	 __HAL_TIM_SET_COMPARE(&IMU_TMP_PWM_HTIM, IMU_TMP_PWM_CHANNEL, pwm);
}
/**
  * @brief  temperature of imu pid control
  * @param[in]: Not used
  * @retval 0
  */
int32_t imu_temp_pid_control(void)
{
  float temp=imu.temp;
  pid_single_loop_control(DEFAULT_IMU_TEMP, &(imu.tmp_pid), temp); // pid control

  if(temp <= (DEFAULT_IMU_TEMP+0.1f) && temp >= (DEFAULT_IMU_TEMP-0.1f)){
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);
	  set_imu_pwm(&imu, imu.tmp_pid.total_out);
	  set_imu_temp_status(&imu, NORMAL);
  }
  else if(temp > DEFAULT_IMU_TEMP + 0.1f){
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
	  set_imu_pwm(&imu, imu.tmp_pid.total_out);
	  set_imu_temp_status(&imu, ABNORMAL);
  }
  else{
	  HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
	  set_imu_pwm(&imu, imu.tmp_pid.total_out);
	  set_imu_temp_status(&imu, ABNORMAL);
  }
  return 0;
}

#endif
