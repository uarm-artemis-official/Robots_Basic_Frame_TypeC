/*
******************************************************************************
* @file           : Referee_App.c
* @brief      	  : Referee system related files
* @created time	  : Jul, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/

#ifndef __REFEREE_APP_C__
#define __REFEREE_APP_C__

// #ifdef __cplusplus
// extern "C" {
// #endif

#include "Referee_App.h"
#include "crc.h"
#include "message_center.h"
// #include "referee_data.h"
#include "string.h"
// #include "usart.h"

// Referee_t referee;

// extern UART_HandleTypeDef huart1;
// extern int16_t referee_parsed_flag;
// extern uint8_t referee_timeout_counter;
// extern uint8_t referee_timeout_check_flag;
// /*
// *  @Referee System Note
// *		JUL, 2023: Use UART3 DMA IT to read the data from referee system intead of freertos task
// *
// * 	Helpful cmd index
// * 		0x0001  Competition status data				 3 Hz
// * 		0x0003  Robot HP data in competition		 3 Hz
// * 		0x0201  Robot status data					10 Hz
// * 		0x0202	Real-time power and heat data       50 Hz
// * 		0x0207  Real-time shoot/launching data		Real-Time(as launching)
// * 		0x0301  Interaction data between robots		10 Hz
// * 		0x0302  interface of the custom controller  30 Hz
// *
// * 		CMD id 0x0301 allows us to draw a customized graphic interface for operators
// *
// * */

// uint8_t ref_tx_frame[MAX_REF_TX_DATA_LEN];  // Data pool

// /**
//  * @brief     main ref sys task function
//  * @param[in] None
//  * @retval    None
//  */
RefereeApp::RefereeApp(IMessageCenter& msg_center, IEventCenter& evt_center,
                       IDebug& debug, IRefUI& ref_ui)
    : message_center(msg_center),
      event_center(evt_center),
      debug(debug),
      ref_ui(ref_ui) {}

void RefereeApp::init() {
    // Initialization code for referee app
    // Receive process initialization
    memset(&(ref.header), 0, sizeof(frame_header_t));
    memset(&(ref.game_status_data), 0, sizeof(game_status_t));
    memset(&(ref.robot_HP_data), 0, sizeof(game_robot_HP_t));
    memset(&(ref.robot_status_data), 0, sizeof(robot_status_t));
    memset(&(ref.power_heat_data), 0, sizeof(power_heat_data_t));
    memset(&(ref.shoot_data), 0, sizeof(shoot_data_t));

    ref_ui.init();  // Initialize referee UI
    // init ui data
    ref.ref_info_data.act_mode = INDPET_MODE;  // Default act mode
    ref.ref_info_data.level = 1;               // Default level
    ref.ref_info_data.super_cap_percent = 0;   // Default super cap percent

    ref.robot_status_data.robot_level = 1;
    ref.ref_cmd_id = IDLE_ID;
    ref.robot_color = UNKOWN;

    // UI sending initialization
}
void RefereeApp::loop() {
    // Main loop for referee app
    // outlines:
    // Read data from referee system (from Topic)
    if (message_center.get_message(REFEREE_IN, ref.ref_rx_frame, 0) == pdTRUE) {
        // Process received data
        read_ref_data();
        non_recv_count = 0;
        // TODO: public referee data as needed
        // message_center.pub_message(REFEREE_OUT, ref.robot_status_data, 0);
    } else {
        // If no data received, increment non-receive count
        non_recv_count++;
        if (non_recv_count > REFEREE_NON_RECV_MAX_COUNT) {
            // Reset referee data if no data received for too long
            // reset();
            non_recv_count = 0;  // Reset the count
        }
    }

    // Update UI if necessary
    draw_all_ui();
}

void RefereeApp::read_ref_data() {
    // Read data from referee system
    if (ref.ref_rx_frame == NULL) {
        // frame is NULL, return
        return;
    }
    /* copy frame header */
    memcpy(&ref.header, ref.ref_rx_frame, HEADER_LEN);

    /* frame header CRC8 verification */
    // FIXME: We don't know if we still need crc8 verification. if not , probably just update the pointer
    if (ref.header.sof == SOF_ID) {
        if (Verify_CRC8_Check_Sum((unsigned char*) &(ref.header), HEADER_LEN) ==
            1) {

            /* successfully verified */
            // ref.ref_cmd_id = (uint16_t)((ref.ref_rx_frame[HEADER_LEN] << 8) | ref.ref_rx_frame[HEADER_LEN + 1]);

            ref.ref_cmd_id = *(
                uint16_t*) (ref.ref_rx_frame +
                            HEADER_LEN);  //point to the addr of the cmd id (rx_frame[6] << 8 | rx_frame[5])

            memcpy(
                ref.ref_data, ref.ref_rx_frame + HEADER_LEN + CMD_LEN,
                sizeof(
                    ref.ref_data));  //pointer to the beginning of the data addr

            /* parse the frame and get referee data */
            switch (ref.ref_cmd_id) {
                case GAME_STAT_ID: {
                    memcpy(&(ref.game_status_data), ref.ref_data,
                           sizeof(game_status_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                case GMAE_HP_ID: {
                    memcpy(&(ref.robot_HP_data), ref.ref_data,
                           sizeof(game_robot_HP_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                case GAME_RESULT_ID: {
                    memcpy(&(ref.game_result_data), ref.ref_data,
                           sizeof(game_result_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                case ROBOT_STAT_ID: {
                    memcpy(&(ref.robot_status_data), ref.ref_data,
                           sizeof(robot_status_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                case POWER_HEAT_ID: {
                    memcpy(&(ref.power_heat_data), ref.ref_data,
                           sizeof(power_heat_data_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                case SHOOT_ID: {
                    memcpy(&(ref.shoot_data), ref.ref_data,
                           sizeof(shoot_data_t));
                    ref.ref_cmd_id = IDLE_ID;
                    break;
                }
                default: {
                    break;
                }
            }

            if (*(ref.ref_rx_frame + HEADER_LEN + CMD_LEN +
                  sizeof(ref.ref_data) + CRC_LEN) ==
                0xA5) {  // Parsed multi-frame in one pack if needed
                read_ref_data();
            }
        }
    } else {
        ref.ref_cmd_id = IDLE_ID;
        // reset();
    }
}

void RefereeApp::draw_all_ui() {
    // Draw all UI elements
    // get information from chassis command data
    ChassisCommandMessage_t chassis_command;

    if (message_center.peek_message(COMMAND_CHASSIS, &chassis_command, 0) ==
        pdTRUE) {
        // Extract act_mode from command_bits (lower 3 bits)
        ref.ref_info_data.act_mode =
            static_cast<BoardActMode_t>(chassis_command.command_bits & 0x7);
    }

    ref.ref_info_data.level =
        (uint32_t) ref.robot_status_data.robot_level;  // Set the level

    ui_sendig_count++;
    if (ui_sendig_count >= 50)  // 50 * 10ms = 500ms = 0.5s sending freq = 2Hz
    {
        // ref_ui.set_ui_data(UI_INFANTRY_MARK, ref.robot_status_data.robot_id,
        //                    ref.ref_info_data);
        // ref_ui.send_ui_data(INTERA_UI_ID, UI_INFANTRY_MARK_LEN,
        //                     UI_INFANTRY_MARK);
        ref_ui.set_ui_data(UI_ROBOT_VAILD_INFO, ref.robot_status_data.robot_id,
                           ref.ref_info_data);
        ref_ui.send_ui_data(INTERA_UI_ID, UI_ROBOT_VAILD_INFO_LEN,
                            UI_ROBOT_VAILD_INFO);
        ui_sendig_count = 0;
    }
    // }
}

// void RefereeApp::reset() {
//     /* Reset DMA */
//     HAL_UART_DMAStop(&huart1);

//     /* Clear rx buffer */
//     // memset(ref.ref_rx_frame, 0, 256);

//     /* Resume UART DMA Receive */
//     HAL_UART_Receive_DMA(&huart1, ref.ref_rx_frame, sizeof(ref.ref_rx_frame));
// }

// /* Drawing the UI */
// uint8_t pack_seq = 0;
// /**
//  * @brief     Referee UI send
//  * @param[in] cmd_id The id sent , usually the ui id 0x0301
//  * @param[in] pdata the data frame to be sent
//  * @param[in] len The length of the actual data frame
//  * @retval    None
//  */
// void referee_send_ui_data(uint16_t cmd_id, uint8_t* p_data, uint16_t len) {

//     uint16_t frame_length =
//         HEADER_LEN + CMD_LEN + len + CRC_LEN;  // length of data frame

//     memset(ref_tx_frame, 0, frame_length);  // Refresh the tx data buffer

//     /* Packing the tx header */
//     ref_tx_frame[0] = SOF_ID;  // First byte of the data pool, 1 byte
//     memcpy(&ref_tx_frame[1], (uint8_t*) &len,
//            sizeof(len));         // Actual data len in the data pool, 2 bytes
//     ref_tx_frame[3] = pack_seq;  // Pack sequence, start from 3, 1 byte
//     Append_CRC8_Check_Sum(
//         ref_tx_frame,
//         HEADER_LEN);  // CRC8 Calibration required by referee system

//     /* Pack cmd id */
//     memcpy(&ref_tx_frame[HEADER_LEN], (uint8_t*) &cmd_id, CMD_LEN);

//     /* Pack actual data frame */
//     memcpy(&ref_tx_frame[HEADER_LEN + CMD_LEN], p_data, len);
//     Append_CRC16_Check_Sum(ref_tx_frame, frame_length);  // CRC16 Calibration

//     if (pack_seq == 0xff)
//         pack_seq = 0;
//     else
//         pack_seq++;

//     HAL_UART_Transmit_DMA(&huart1, ref_tx_frame, frame_length);
// }

// void referee_set_ui_data(Referee_t* ref, referee_ui_t ui_type) {

//     if (ref->robot_status_data.robot_id == 3 ||
//         ref->robot_status_data.robot_id == 4 ||
//         ref->robot_status_data.robot_id == 103 ||
//         ref->robot_status_data.robot_id == 104) {  // Infantry
//         ref->ui_intrect_data.sender_id = ref->robot_status_data.robot_id;
//         switch (ref->robot_status_data.robot_id) {
//             case 3:
//                 ref->ui_intrect_data.receiver_id = 0x0103;
//                 break;  // Red team #3 infantry client
//             case 4:
//                 ref->ui_intrect_data.receiver_id = 0x0104;
//                 break;  // Red team #4 infantry client
//             case 103:
//                 ref->ui_intrect_data.receiver_id = 0x0167;
//                 break;  // Blue team #3 infantry client
//             case 104:
//                 ref->ui_intrect_data.receiver_id = 0x0168;
//                 break;  // Blue team #4 infantry client
//         }

//         switch (ui_type) {
//             case UI_ROBOT_ACT_MODE:
//                 referee_general_draw_act_mode(ref);
//                 break;
//             /* Hero Draw marks */
//             case UI_INFANTRY_MARK:
//                 referee_infantry_draw_marks(ref);
//                 break;
//             default:
//                 break;
//         }
//     } else if (ref->robot_status_data.robot_id == 1 ||
//                ref->robot_status_data.robot_id == 101) {  // Hero
//         /* Set up */
//         ref->ui_intrect_data.sender_id = ref->robot_status_data.robot_id;
//         /* Set up client ID */
//         switch (ref->robot_status_data.robot_id) {
//             case 1:
//                 ref->ui_intrect_data.receiver_id = 0x0101;
//                 break;  // Red team #1 Hero client
//             case 101:
//                 ref->ui_intrect_data.receiver_id = 0x0165;
//                 break;  // Blue team #1 Hero client
//         }

//         switch (ui_type) {
//             case UI_ROBOT_ACT_MODE:
//                 referee_general_draw_act_mode(ref);
//                 break;
//             /* Hero Draw marks */
//             case UI_HERO_MARK:
//                 referee_hero_draw_marks(ref);
//                 break;
//             default:
//                 break;
//         }
//     }
// }

// #ifdef __cplusplus
// }
// #endif
#endif
