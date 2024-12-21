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

#include "Timer_App.h"
#include "Shoot_App.h"
#include "public_defines.h"

int32_t test_value = 0 * 100;


/**
* @brief  Timer app used to update the CAN data
* 		  Also reserve for other real time tasks
* @param  Not used
* @retval None
*/

/* Task execution time (per loop): 1ms */
//FIXME: this task takes too much time to run, try to optimize it within 2-3ms
void Timer_Task_Func(void const * argument){

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // task exec period 1ms

	/* Simple way to init the lk motor */
	for (uint8_t i = 0; i < LK_MOTOR_COUNT; i++) {
	        LK_motor_init(i);
	    }

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

//	LK_motor_stop(&hcan1, KL_MOTOR_TX_STDID+1);

	for (;;){

		lk_motor[0].tx_data = test_value;
		LK_motor_send(&hcan1, lk_motor[0].send_id, lk_motor[0].motor_cmd, lk_motor[0].tx_data);

		//FIXME: may put this read fucntion to can pending callback function
		Motor_Data_Read(&hcan1);
		/* CAN data  */
		if(board_status == CHASSIS_BOARD){
			Motor_Data_Send(&hcan1, MOTOR_3508_STDID,
							motor_data[wheel_id1].tx_data,
							motor_data[wheel_id2].tx_data,
							motor_data[wheel_id3].tx_data,
							motor_data[wheel_id4].tx_data);
		}
		else if(board_status == GIMBAL_BOARD){
			Motor_Data_Send(&hcan1, MOTOR_6020_STDID,
							motor_data[yaw_id].tx_data,
							motor_data[pitch_id].tx_data,
							motor_data[mag_2006_id].tx_data,
							0);
#ifdef USE_CAN_FRIC
			Motor_Data_Send(&hcan1, MOTOR_3508_STDID,
							motor_data[fric_left_id].tx_data,
							motor_data[fric_right_id].tx_data,
							0,
							0);
#endif
		}

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


