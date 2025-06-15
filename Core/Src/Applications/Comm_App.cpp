/*
******************************************************************************
* @file           : Comm_App.c
* @brief          : communication real time task between boards
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/
#include "Comm_App.h"
#include "apps_defines.h"
#include "string.h"
#include "uarm_lib.h"
#include "uarm_os.h"

CommApp::CommApp(IMessageCenter& message_center_ref, IDebug& debug_ref,
                 ICanComm& can_comm_ref)
    : message_center(message_center_ref),
      debug(debug_ref),
      can_comm(can_comm_ref) {}

void CommApp::init() {
    board_status = debug.get_board_status();
}

void CommApp::loop() {
    CANCommMessage_t outgoing_message, incoming_message;
    BaseType_t new_send_message =
        message_center.get_message(COMM_OUT, &outgoing_message, 0);
    BaseType_t new_receive_message =
        message_center.get_message(COMM_IN, &incoming_message, 0);

    if (new_send_message == pdTRUE) {
        can_comm.can_transmit_comm_message(outgoing_message.data,
                                           outgoing_message.topic_name);
    }

    if (new_receive_message == pdTRUE) {
        switch (incoming_message.topic_name) {
            case REF_INFO:
                // TODO: Implement
                break;
            case GIMBAL_REL_ANGLES: {
                float rel_angles[2];
                memcpy(rel_angles, incoming_message.data, sizeof(float) * 2);
                message_center.pub_message(GIMBAL_REL_ANGLES, rel_angles);
            } break;
            case RC_INFO: {
                RCInfoMessage_t rc_info;
                memset(&rc_info, 0, sizeof(RCInfoMessage_t));
                memcpy(rc_info.channels, &(incoming_message.data[4]),
                       sizeof(int16_t) * 2);
                memcpy(rc_info.modes, incoming_message.data,
                       sizeof(uint8_t) * 3);
                message_center.pub_message(RC_INFO, &rc_info);
            } break;
            case PLAYER_COMMANDS:
                // TODO: Implement
                break;
            default:
                break;
        }
    }
};

/***************************** CAN COMM BEGAIN ************************************/
/**
* @brief CAN commnication message subscription
* @param None
* @retval None
*/
///**
//* @brief CAN commnication struct initialization
//* @param None
//* @retval None
//*/
//void can_comm_reset_config(BoardComm_t *comm){
//	comm->comm_mode = CAN_COMM_MODE;
//	comm->can_comm.comm_id = IDLE_COMM_ID;
//	/* init rx buffer */
//
//	for(int i = 0; i < TOTAL_COMM_ID; i++) {
//		if(i<4)
//			comm->can_comm.tx_data[i] = 0;
//		comm->can_comm.rx_data[i][0] = 0;
//		comm->can_comm.rx_data[i][1] = 0;
//		comm->can_comm.rx_data[i][2] = 0;
//		comm->can_comm.rx_data[i][3] = 0;
//	}
//    comm->can_comm.can_send_comm_data = can_send_comm_data;
//    comm->can_comm.can_recv_comm_data = can_recv_comm_data;
//
//    /* init subscription list */
//    memset(&comm->sub_list, 0, sizeof(CommMessageSublist_t));
//	/* init fifo queue */
//	queueM_init(&canqm);
//}

/**
* @brief CAN commnication send data float to int16
* @param comm: main comm app struct
* 		 scale_factor: corresponding scale_factor
*/
// TODO: Look into better float to int16_t conversion.
// void process_tx_data_ftoi16(const float* input_data, int16_t* output_data,
//                             int length, float scale_factor) {
//     for (int i = 0; i < length; i++) {
//         output_data[i] = (int16_t) (input_data[i] * scale_factor);
//     }
// }

/**
* @brief CAN commnication recving data int16 to float
* @param comm: main comm app struct
* 		 scale_factor: corresponding scale_factor
*/
// TODO: Look into better int16_t to float conversion.
// void process_rx_data_i16tof(float* output_buffer, int16_t input_data[4],
//                             float scale_factor) {
//     for (int i = 0; i < 4; i++) {
//         output_buffer[i] = ((float) input_data[i]) / scale_factor;
//     }
// }

/**
* @brief CAN commnication receiving function, activated for CAN2 comms
* @param CAN_HandleTypeDef object: A can pointer refer to a CAN structure
* 		 int32_t* send_data: The data is ready to be sent
* @retval None
*/
// void can_recv_comm_data(CAN_HandleTypeDef* hcan, uint32_t data_len, int16_t (*rx_buffer)[TOTAL_COMM_ID][4]) {
// 	uint8_t comm_temp_rx_buffer[8];
// 	for(int i=0;i<TOTAL_COMM_ID;i++){
// 		memcpy(comm_temp_rx_buffer, can_comm_rx[i].comm_rx_buffer, data_len);
// 		(*rx_buffer)[i][0] = (int16_t)(comm_temp_rx_buffer[0] << 8 | comm_temp_rx_buffer[1]);
// 		(*rx_buffer)[i][1] = (int16_t)(comm_temp_rx_buffer[2] << 8 | comm_temp_rx_buffer[3]);
// 		(*rx_buffer)[i][2] = (int16_t)(comm_temp_rx_buffer[4] << 8 | comm_temp_rx_buffer[5]);
// 		(*rx_buffer)[i][3] = (int16_t)(comm_temp_rx_buffer[6] << 8 | comm_temp_rx_buffer[7]);
// 	}
// }