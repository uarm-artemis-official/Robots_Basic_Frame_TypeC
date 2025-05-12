#include "stddef.h"
#include "stdint.h"

#include "can.h"
#include "can_isr.h"
#include "lk_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "motors.h"


typedef struct {
    uint32_t stdId;
    Motor_Feedback_t feedback;
} CAN_Rx_Buffer_t;


static CAN_Rx_Buffer_t can_rx_buffer[8] = {
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
    { .stdId = 0, .feedback = { 0 } },
};


static uint8_t get_free_buffer(uint32_t stdId) {
    uint8_t free_index = -1;
    for (int i = 0; i < 8; i++) {
        if (can_rx_buffer[i].stdId == 0) free_index = 0;
        if (can_rx_buffer[i].stdId == stdId) {
            return i;
        }
    }
    return free_index;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef rx_header;
	rx_header.StdId = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	if (hcan == &hcan1) {
        uint8_t buffer[8];
        uint8_t buffer_index = get_free_buffer(rx_header.StdId);
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buffer);
        can_rx_buffer[buffer_index].stdId = rx_header.StdId;
        parse_feedback(rx_header.StdId, buffer, &(can_rx_buffer[buffer_index].feedback));
	}
	if (hcan == &hcan2) {
        CANCommMessage_t incoming_message;
		incoming_message.topic_name = rx_header.StdId;
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, incoming_message.data);
        // TODO: Store messages in RTOS queue and implement API for accessing messages.
		// pub_message_from_isr(COMM_IN, &incoming_message, NULL);
	}
}