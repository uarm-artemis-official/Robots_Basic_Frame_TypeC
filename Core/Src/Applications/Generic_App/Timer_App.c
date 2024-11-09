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

/**
* @brief  Timer app used to update the CAN data
* 		  Also reserve for other real time tasks
* @param  Not used
* @retval None
*/
static Subscriber_t *motor_set_sub;
static int32_t curr_tx_buffer[MOTOR_TX_BUFFER_SIZE];
static int32_t new_tx_buffer[MOTOR_TX_BUFFER_SIZE];

void Timer_Task_Func(void const * argument){

	/* set task exec period */
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint8_t board_status = *(uint8_t*) argument;
	motor_set_sub = register_sub("MOTOR_SET", MOTOR_TX_BUFFER_SIZE * sizeof(int32_t));
	memset(curr_tx_buffer, 0, 64);

	for (;;) {
		/* CAN data  */
		uint8_t received_new_message = get_message(motor_set_sub, new_tx_buffer);
		if (received_new_message) {
			for (int i = 0; i < MOTOR_TX_BUFFER_SIZE; i++) {
				if (-30000 < new_tx_buffer[i] && new_tx_buffer[i] < 30001) {
					curr_tx_buffer[i] = new_tx_buffer[i];
				}
			}

			if(board_status == CHASSIS_BOARD) {
				set_motor_voltage(&hcan1, MOTOR_3508_STDID,
					can_tx_buffer[wheel_id1],
					can_tx_buffer[wheel_id2],
					can_tx_buffer[wheel_id3],
					can_tx_buffer[wheel_id4]);
			} else if(board_status == GIMBAL_BOARD) {
				set_motor_voltage(&hcan1, MOTOR_6020_STDID,
					can_tx_buffer[yaw_id],
					can_tx_buffer[pitch_id],
					can_tx_buffer[mag_2006_id],
					0);
#ifdef USE_CAN_FRIC
				set_motor_voltage(&hcan1, MOTOR_3508_STDID,
					motor_data[fric_left_id].tx_data,
					motor_data[fric_right_id].tx_data,
					0,
					0);
#endif
			}
		}

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


