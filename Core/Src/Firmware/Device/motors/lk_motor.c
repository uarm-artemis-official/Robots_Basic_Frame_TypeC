/*******************************************************************************
* @file           : motor.c
* @created time	  : Dec, 2020
* @Creator        : Haoran, Xiuhong (Amara Yu)
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include "lk_motor.h"
#include <string.h>
#include "motor_defines.h"

/**
  * @brief     LK motor init
  * @param[in] lk_motor_id  actual sending id - 1
  * @retval    None
  */
// void lk_motor_init(uint8_t lk_motor_id){
// 	lk_motor[lk_motor_id].send_id = LK_MOTOR_TX_STDID + lk_motor_id + 1; // lk_motor_id starts from 0 but should start from 1
// 	lk_motor[lk_motor_id].motor_cmd = LK_CMD_ML_ANGLE; // Default to multi-loop angle control
// 	lk_motor[lk_motor_id].tx_data = 0; // FIXME: Naive settings, may apply for ramp initialization like gimbal
// }

/**
  * @brief     LK motor send control command
  * @param[in] id :   Motor id, usually 0x140 + No. of the motor
  * @param[in] control_cmd : Command to send
  * @param[in] sendValue,
  * @retval    None
  */
void lk_motor_send(uint32_t id, LK_Motor_Command_t control_cmd,
                   int32_t sendValue) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = id;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = control_cmd;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = *((uint8_t*) &sendValue + 0);
    tx_data[5] = *((uint8_t*) &sendValue + 1);
    tx_data[6] = *((uint8_t*) &sendValue + 2);
    tx_data[7] = *((uint8_t*) &sendValue + 3);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,
                         (uint32_t*) CAN_TX_MAILBOX1);
}

/**
  * @brief     Read feedback from the LK motor sensor
  * @param[in] can1/can2 type header
  * @retval    None
  */
void lk_motor_parse_feedback(const uint8_t rx_buffer[8], void* fb_ptr) {
    switch (rx_buffer[0]) {
        case LK_MOTOR_READ_SL_FB:
            uint8_t* sl_feedback = (uint8_t*) fb_ptr;
            sl_feedback[0] = rx_buffer[4];
            sl_feedback[1] = rx_buffer[5];
            sl_feedback[2] = rx_buffer[6];
            sl_feedback[3] = rx_buffer[7];
            break;
        case LK_MOTOR_READ_FB_DATA:
            Motor_Feedback_t* raw_feedback_ptr = (Motor_Feedback_t*) fb_ptr;
            raw_feedback_ptr->rx_angle =
                (int16_t) (rx_buffer[7] << 8 | rx_buffer[6]);
            raw_feedback_ptr->rx_rpm =
                (int16_t) (rx_buffer[5] << 8 | rx_buffer[4]);
            raw_feedback_ptr->rx_current =
                (int16_t) (rx_buffer[3] << 8 | rx_buffer[2]);
            raw_feedback_ptr->rx_temp = (int16_t) (rx_buffer[1]);
            break;
        default:
            return;
    }
}

void lk_motor_send_multi_loop(uint32_t id, uint16_t max_speed, uint32_t angle) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = id;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = LK_CMD_ML_ANGLE_WITH_SPEED;
    tx_data[1] = 0x00;
    tx_data[2] = *((uint8_t*) (&max_speed) + 0);
    tx_data[3] = *((uint8_t*) (&max_speed) + 1);
    tx_data[4] = *((uint8_t*) (&angle) + 0);
    tx_data[5] = *((uint8_t*) (&angle) + 1);
    tx_data[6] = *((uint8_t*) (&angle) + 2);
    tx_data[7] = *((uint8_t*) (&angle) + 3);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,
                         (uint32_t*) CAN_TX_MAILBOX1);
}

void lk_motor_send_single_loop(uint32_t id, uint8_t spin_direction,
                               uint16_t max_speed, uint32_t angle) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = id;
    tx_header.ExtId = 0x00;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    tx_data[0] = LK_CMD_SL_ANGLE_WITH_SPEED;
    tx_data[1] = spin_direction;
    tx_data[2] = *((uint8_t*) (&max_speed) + 0);
    tx_data[3] = *((uint8_t*) (&max_speed) + 1);
    tx_data[4] = *((uint8_t*) (&angle) + 0);
    tx_data[5] = *((uint8_t*) (&angle) + 1);
    tx_data[6] = *((uint8_t*) (&angle) + 2);
    tx_data[7] = *((uint8_t*) (&angle) + 3);

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,
                         (uint32_t*) CAN_TX_MAILBOX1);
}