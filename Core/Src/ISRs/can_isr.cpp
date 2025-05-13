#include "stddef.h"
#include "stdint.h"
#include "string.h"

#include "can.h"
#include "can_isr.h"
#include "dji_motor.h"
#include "lk_motor.h"
#include "message_center.h"
#include "motors.h"

static MotorReadMessage_t read_message;
static MessageCenter& message_center = MessageCenter::get_instance();

void init_can_isr() {
    memset(&read_message, 0, sizeof(MotorReadMessage_t));
}

static uint8_t get_free_buffer(uint32_t stdId) {
    uint8_t free_index = 0xff;
    for (int i = 0; i < 8; i++) {
        if (read_message.can_ids[i] == 0)
            free_index = i;
        if (read_message.can_ids[i] == stdId) {
            return i;
        }
    }
    return free_index;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CAN_RxHeaderTypeDef rx_header;
    rx_header.StdId =
        (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >>
        CAN_TI0R_STID_Pos;
    if (hcan == &hcan1) {
        uint8_t buffer[8];
        uint8_t buffer_index = get_free_buffer(rx_header.StdId);
        if (buffer_index < 8) {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buffer);
            read_message.can_ids[buffer_index] =
                static_cast<Motor_CAN_ID_t>(rx_header.StdId);
            Motors::parse_feedback(rx_header.StdId, buffer,
                                   &(read_message.feedback[buffer_index]));
            message_center.pub_message_from_isr(MOTOR_READ, &read_message,
                                                NULL);
        }
    }
    if (hcan == &hcan2) {
        CANCommMessage_t incoming_message;
        incoming_message.topic_name = rx_header.StdId;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                             incoming_message.data);
        // TODO: Store messages in RTOS queue and implement API for accessing messages.
        // pub_message_from_isr(COMM_IN, &incoming_message, NULL);
    }
}