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
#include "can_comm.h"
#include "debug.h"
#include "message_center.h"
#include "public_defines.h"
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

// TickType_t xLastWakeTime;
// const TickType_t xFrequency = pdMS_TO_TICKS(COMM_TASK_EXEC_TIME);
// xLastWakeTime = xTaskGetTickCount();

// for (;;) {
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
// }
/***************************** CAN COMM BEGAIN ************************************/
/**
* @brief CAN commnication message subscription
* @param None
* @retval None
*/
//void can_comm_subscribe_process(void){
//	if(board_status == CHASSIS_BOARD){
//		comm_subscribe(&chassis_comm.sub_list, COMM_REMOTE_CONTROL, Transmitter);
//		comm_subscribe(&chassis_comm.sub_list, COMM_PC_CONTROL, Transmitter);
//		comm_subscribe(&chassis_comm.sub_list, COMM_EXT_PC_CONTROL, Transmitter);
//		comm_subscribe(&chassis_comm.sub_list, COMM_GIMBAL_ANGLE, Receiver);
//	}
//	else if(board_status == GIMBAL_BOARD){
//		comm_subscribe(&gimbal_comm.sub_list, COMM_GIMBAL_ANGLE, Transmitter);
//		comm_subscribe(&gimbal_comm.sub_list, COMM_REMOTE_CONTROL, Receiver);
//		comm_subscribe(&gimbal_comm.sub_list, COMM_PC_CONTROL, Receiver);
//		comm_subscribe(&gimbal_comm.sub_list, COMM_EXT_PC_CONTROL, Receiver);
//	}
//}
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
/***************************** CAN COMM END ************************************/
/* Since we have multiple can comm works in the future , there is necessity that apply
 * FIFO Queue management of our CAN2 data pool. This function has been moved to algo*/
/***************************** UART COMM BEGAIN ************************************/
/**
* @brief uart commnication process
* @param None
* @retval None
*/
//void usart_comm_process(void){
//	for(;;){
//		if(capture_flag == 0){
//			comm_pack.vision.target_num = 0;
//		}
//		else{
//			osDelay(3000);
//			if(capture_flag == 1)
//				capture_flag = 0;
//		}
//	}
//}
#ifdef USE_UART_DMA
static uint8_t temp_buffer[PACKLEN + 1];
/**
* @brief uart global interrupt function
* @param UART_HandleTypeDef object refered to a uart structure
* @retval None
*/
//FIXME: Use DMA to trigger UART6 and UART7
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    // When enter this callback function, the variable pdata has been filled with the received data.
    // Thus parse it directly.
    HAL_GPIO_WritePin(LD_E_GPIO_Port, LD_E_Pin, RESET);
    if (huart == &huart6 && board_status == CHASSIS_BOARD) {
        HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, RESET);
        //		  comm_pack=parse_all(pdata);
        strncpy(temp_buffer, pdata, PACKLEN);
        comm_pack.vision = parse_all(temp_buffer);
        abs_yaw = angle_preprocess(&motor_data[5], comm_pack.vision.yaw_data,
                                   1.8, YAW_MOTOR);
        //		  abs_pitch=angle_preprocess(&motor_data[4], comm_pack.pitch_data, 1, PITCH_MOTOR);
        temp_buffer[sizeof(temp_buffer) - 1] = '\0';
        HAL_UART_Transmit(&huart7, (char*) temp_buffer, strlen(temp_buffer),
                          1000);
        HAL_UART_Receive_IT(&huart6, (char*) pdata, PACKLEN);
        capture_flag = 1;
    } else if (huart == &huart6 && board_status == GIMBAL_BOARD) {
        comm_pack.vision = parse_all(pdata);
        //		  abs_yaw=angle_preprocess(&motor_data[5], comm_pack.yaw_data, 1, YAW_MOTOR);
        abs_pitch = angle_preprocess(
            &motor_data[4], comm_pack.vision.pitch_data, 1, PITCH_MOTOR);
        HAL_GPIO_WritePin(LD_F_GPIO_Port, LD_F_Pin, RESET);
        HAL_UART_Receive_IT(&huart6, (char*) pdata, PACKLEN);
        capture_flag = 1;
    } else {
        HAL_GPIO_WritePin(LD_F_GPIO_Port, LD_F_Pin, SET);
        capture_flag = 0;
    }
    // Enable the uart interrupt again
}

/* no sub comm func for backing up */
void can_comm_process_nosub(void) {
    Comm_t comm_pack;
    /* can comm tasks */
    if (board_status == CHASSIS_BOARD) {
        //reset the comm struct configure
        //		can_comm_reset_config(&chassis_comm, IDLE_COMM_ID);
        for (;;) {
            /* send tasks */
            if (comm_pack.comm_rc.send_flag == 1) {
                memcpy(&(chassis_comm.can_comm.tx_data),
                       &(comm_pack.comm_rc.rc_data),
                       sizeof(comm_pack.comm_rc.rc_data));
                chassis_comm.can_comm.can_send_comm_data(
                    &hcan2, chassis_comm.can_comm.tx_data, RC_COMM_ID);
                comm_pack.comm_rc.send_flag =
                    0;  //reset flag to avoid message flooding
            }

            /* recv tasks */
            chassis_comm.can_comm.can_recv_comm_data(
                &hcan2, 8, chassis_comm.can_comm.rx_data);
            /* relative angle for attitude breakdown */
            if (can_comm_rx[ANGLE_IDX].comm_id == ANGLE_COMM_ID) {
                process_rx_data(&chassis_comm, comm_pack.comm_ga.angle_data,
                                ANGLE_COMM_SCALE_FACTOR, ANGLE_IDX);
                chassis.gimbal_yaw_rel_angle =
                    -comm_pack.comm_ga
                         .angle_data[0];  //can_rx_scale_buffer[ANGLE_IDX][0];
                chassis.gimbal_yaw_abs_angle =
                    comm_pack.comm_ga
                        .angle_data[1];  //can_rx_scale_buffer[ANGLE_IDX][1];
                can_comm_rx[ANGLE_IDX].comm_id =
                    0;  //reset id to avoid message flooding
            }
            osDelay(1);
        }
    } else if (board_status == GIMBAL_BOARD) {
        //		can_comm_reset_config(&gimbal_comm, IDLE_COMM_ID);
        for (;;) {
            /* send tasks */
            /*yaw relative angle can_tx_scale_buffer[ANGLE_IDX]*/
            if (comm_pack.comm_ga.send_flag == 1) {
                process_tx_data(comm_pack.comm_ga.angle_data,
                                gimbal_comm.can_comm.tx_data, 4,
                                ANGLE_COMM_SCALE_FACTOR);
                gimbal_comm.can_comm.can_send_comm_data(
                    &hcan2, gimbal_comm.can_comm.tx_data, ANGLE_COMM_ID);
                comm_pack.comm_ga.send_flag =
                    0;  //reset flag to avoid message flooding
            }

            /* recv tasks */
            gimbal_comm.can_comm.can_recv_comm_data(
                &hcan2, 8, gimbal_comm.can_comm.rx_data);
            /* relative angle for attitude breakdown */
            if (can_comm_rx[RC_IDX].comm_id == RC_COMM_ID) {
                rc.ctrl.ch0 = gimbal_comm.can_comm.rx_data[RC_IDX][0];
                rc.ctrl.ch1 = gimbal_comm.can_comm.rx_data[RC_IDX][1];
                rc.ctrl.s1 = gimbal_comm.can_comm.rx_data[RC_IDX][2];
                rc.ctrl.s2 = gimbal_comm.can_comm.rx_data[RC_IDX][3];
                can_comm_rx[RC_IDX].comm_id =
                    0;  //reset id to avoid message flooding
            }
            osDelay(1);
        }
    }
}

#endif

/*****************************  UART COMM END  ************************************/
/* The UART comm between boards are being on hold since ethe CAN comm can be more
 * reliable and fast, and we will use UART DMA to save cpu's arithmetic power */
/********************************  COMM END  ************************************/
