/*******************************************************************************
* @file           : motor.c
* @created time	  : Dec, 2020
* @Creator        : Haoran, Xiuhong (Amara Yu)
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include <string.h>
#include "motor.h"
#include "pid.h"
#include "math.h"
#include "public_defines.h"


/**
  * @brief     LK motor init
  * @param[in] lk_motor_id  actual sending id - 1
  * @retval    None
  */
void LK_motor_init(uint8_t lk_motor_id){
	lk_motor[lk_motor_id].send_id = LK_MOTOR_TX_STDID + lk_motor_id + 1; // lk_motor_id starts from 0 but should start from 1
	lk_motor[lk_motor_id].motor_cmd = LK_CMD_ML_ANGLE; // Default to multi-loop angle control
	lk_motor[lk_motor_id].tx_data = 0; // FIXME: Naive settings, may apply for ramp initialization like gimbal
}

/**
  * @brief     LK motor send control command
  * @param[in] hcan : CAN instance for sending the command
  * @param[in] id :   Motor id, usually 0x140 + No. of the motor
  * @param[in] control_cmd : Command to send
  * @param[in] sendValue,
  * @retval    None
  */
void LK_motor_send(CAN_HandleTypeDef* hcan, uint32_t id, LK_Motor_Command_t control_cmd, int32_t sendValue){
	CAN_TxHeaderTypeDef  tx_header;
	uint8_t				 tx_data[8];

	tx_header.StdId = id;
	tx_header.ExtId = 0x00;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	tx_data[0] = control_cmd;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	tx_data[4] = *((uint8_t *)&sendValue + 0);
	tx_data[5] = *((uint8_t *)&sendValue + 1);
	tx_data[6] = *((uint8_t *)&sendValue + 2);
	tx_data[7] = *((uint8_t *)&sendValue + 3);

	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX1);
}

/**
  * @brief     Read feedback from the LK motor sensor
  * @param[in] can1/can2 type header
  * @retval    None
  */
void LK_motor_read(uint8_t **rx_buffer) {
	uint8_t LKmotorStatus[LK_MOTOR_COUNT];
	for (int i=0; i<LK_MOTOR_COUNT; i++){
		memcpy(LKmotorStatus, rx_buffer[8+i], 8);// 0-7 is for RM motor , so start from 8
		lk_motor[i].motor_feedback.rx_angle	  = (int16_t)(LKmotorStatus[7] << 8 | LKmotorStatus[6]);
		lk_motor[i].motor_feedback.rx_rpm	  = (int16_t)(LKmotorStatus[5] << 8 | LKmotorStatus[4]);
		lk_motor[i].motor_feedback.rx_current = (int16_t)(LKmotorStatus[3] << 8 | LKmotorStatus[2]);
		lk_motor[i].motor_feedback.rx_temp	  = (int16_t)(LKmotorStatus[1]);
	}
}




