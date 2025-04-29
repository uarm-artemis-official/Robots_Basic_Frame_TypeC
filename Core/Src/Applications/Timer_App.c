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
	/* Simple way to init the lk motor */
//	for (uint8_t i = 0; i < LK_MOTOR_COUNT; i++) {
//		LK_motor_init(i);
//	}

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

//	LK_motor_stop(&hcan1, KL_MOTOR_TX_STDID+1);

	for (;;){
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

/**
 *
 */

/* This function activates whenever the RxFifo receives a message
 * The StdId is obtained from the can message, then it is written into the buffer array (it is an array of arrays)
 * To figure out which motor it is for the read/write functions, we will refer to a table - see notes from March 25, 2021
 * There may be a better table later
*/
can_comm_rx_t can_comm_rx[7] = {
    {0, {0, 0, 0, 0, 0, 0, 0, 0}}, // IDLE_COMM_ID
    {0, {0, 0, 0, 0, 0, 0, 0, 0}}, // ANGLE_COMM_ID
    {0, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0, {0, 0, 0, 0, 0, 0, 0, 0}},
    {0, {0, 0, 0, 0, 0, 0, 0, 0}},
	{0, {0, 0, 0, 0, 0, 0, 0, 0}},
};
//FIXME: don't why it cannot detect MOTOR_COUNT and TOTAL_COMM_ID
static uint8_t can_rx_buffer[8][8]; // Motor count + maximum once sending bytes
static Motor_Feedback_t motor_feedback[MOTOR_COUNT];
static CANCommMessage_t incoming_message;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	rx_header.StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	if (hcan == &hcan1) {
		uint8_t idx = rx_header.StdId-CAN_RX_ID_START;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_buffer[idx]);

		parse_motor_feedback(can_rx_buffer[idx], &(motor_feedback[idx]));
		pub_message_from_isr(MOTOR_READ, motor_feedback, NULL);
	}
	if (hcan == &hcan2) {
		incoming_message.topic_name = rx_header.StdId;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, incoming_message.data);
		pub_message_from_isr(COMM_IN, &incoming_message, NULL);
	}
}