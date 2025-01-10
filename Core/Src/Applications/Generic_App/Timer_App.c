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
// TODO: Limit motor_tx_buffer to int16_t instead of int32_t.
static int32_t motor_tx_buffer[MOTOR_TX_BUFFER_SIZE];
static MotorSetMessage_t motor_tx_message;
static BoardStatus_t board_status;

void Timer_Task_Func(void const * argument){

	/* set task exec period */
	const TickType_t xFrequency = pdMS_TO_TICKS(TIMER_TASK_EXEC_TIME);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	board_status = *(BoardStatus_t*) argument;
	memset(motor_tx_buffer, 0, 64);

	for (;;) {
		/* CAN data  */

		BaseType_t received_new_message = get_message(MOTOR_SET, &motor_tx_message, 0);
		if (received_new_message == pdTRUE) {
			for (int i = 0; i < MOTOR_TX_BUFFER_SIZE; i++) {
				if (((1 << i) & motor_tx_message.data_enable) == 0) continue;
				if (-30000 <= motor_tx_message.motor_can_volts[i] && motor_tx_message.motor_can_volts[i] <= 30000) {
					motor_tx_buffer[i] = motor_tx_message.motor_can_volts[i];
				}
			}

			if(board_status == CHASSIS_BOARD) {
				// TODO: Figure out why this is setting "voltage" when PID is calculated based on current.
#ifdef TIMER_CHASSIS_WHEELS_ENABLE
				set_motor_voltage(&hcan1, MOTOR_3508_STDID,
					motor_tx_buffer[CHASSIS_WHEEL1_CAN_ID],
					motor_tx_buffer[CHASSIS_WHEEL2_CAN_ID],
					motor_tx_buffer[CHASSIS_WHEEL3_CAN_ID],
					motor_tx_buffer[CHASSIS_WHEEL4_CAN_ID]);
#endif
			} else if(board_status == GIMBAL_BOARD) {
#ifdef TIMER_GIMBAL_MAG_ENABLE
				set_motor_voltage(&hcan1, MOTOR_6020_STDID,
					motor_tx_buffer[GIMBAL_YAW_CAN_ID],
					motor_tx_buffer[GIMBAL_PITCH_CAN_ID],
					motor_tx_buffer[SHOOT_LOADER_CAN_ID],
					0);
#endif

#ifdef TIMER_FRIC_WHEELS_ENABLE
				const int32_t limit = 15000;
				int32_t temp_left_fric = motor_tx_buffer[SHOOT_LEFT_FRIC_CAN_ID];
				int32_t temp_right_fric = motor_tx_buffer[SHOOT_RIGHT_FRIC_CAN_ID];
				if (temp_left_fric > limit) temp_left_fric = limit;
				if (temp_left_fric < -limit) temp_left_fric = -limit;
				if (temp_right_fric > limit) temp_right_fric = limit;
				if (temp_right_fric < -limit) temp_right_fric = -limit;

				set_motor_voltage(&hcan1, MOTOR_3508_STDID,
					temp_left_fric,
					temp_right_fric,
					0,
					0);
#endif
			}
		}

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


