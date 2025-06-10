#include "dji_motor.h"

extern CAN_HandleTypeDef hcan1;

/**
  * @brief     Read feedback from the motor sensor
  * @param[in] can1/can2 type header
  * @retval    None
  */
void dji_motor_get_raw_feedback(const uint8_t* can_data,
                                Motor_Feedback_t* motor_feedback) {
    motor_feedback->rx_angle =
        (((int16_t) can_data[0]) << 8) | ((int16_t) can_data[1]);
    motor_feedback->rx_rpm =
        (((int16_t) can_data[2]) << 8) | ((int16_t) can_data[3]);
    motor_feedback->rx_current =
        (((int16_t) can_data[4]) << 8) | ((int16_t) can_data[5]);
    motor_feedback->rx_temp = (int16_t) (can_data[6]);
}

void __dji_motor_send(int32_t id, int32_t d1, int32_t d2, int32_t d3,
                      int32_t d4, uint8_t use_can2) {
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 0x08;

    tx_data[0] = d1 >> 8;
    tx_data[1] = d1;
    tx_data[2] = d2 >> 8;
    tx_data[3] = d2;
    tx_data[4] = d3 >> 8;
    tx_data[5] = d3;
    tx_data[6] = d4 >> 8;
    tx_data[7] = d4;

    if (use_can2 == 1) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,
                             (uint32_t*) CAN_TX_MAILBOX0);

    } else {
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,
                             (uint32_t*) CAN_TX_MAILBOX0);
    }
}

void dji_motor_send(int32_t id, int32_t d1, int32_t d2, int32_t d3,
                    int32_t d4) {
    __dji_motor_send(id, d1, d2, d3, d4, 0);
}
