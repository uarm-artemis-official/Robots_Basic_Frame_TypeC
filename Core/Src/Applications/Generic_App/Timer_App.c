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

extern Shoot_t shoot;
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

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	for (;;){

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
							motor_data[pitch_id].tx_data,
							motor_data[yaw_id].tx_data,
							0,
							0);
#ifdef USE_CAN_FRIC
			if(shoot.shoot_act_mode == SHOOT_CEASE)
				motor_data[mag_3508_id].tx_data = 0;
			Motor_Data_Send(&hcan1, MOTOR_3508_STDID,
							motor_data[fric_left_id].tx_data,
							motor_data[fric_right_id].tx_data,
							motor_data[mag_3508_id].tx_data,
							0);
#endif
		}

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


