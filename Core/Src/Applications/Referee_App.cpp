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
}

void RefereeApp::loop() {
    // Main loop for referee app
    // outlines:
    // Read data from referee system (from Topic)
    if (message_center.get_message(REFEREE_IN, ref.ref_rx_frame, 0) == pdTRUE) {
        // Process received data
        read_ref_data();
        non_recv_count = 0;

        RefereeInfoMessage_t ref_message;
        ref_message.robot_id = ref.robot_status_data.robot_id;
        ref_message.robot_level = ref.robot_status_data.robot_level;
        ref_message.shoot_barrel_cooling_rate =
            ref.robot_status_data.shooter_barrel_cooling_value;
        ref_message.chassis_power_limit =
            ref.robot_status_data.chassis_power_limit;
        message_center.pub_message(REFEREE_OUT, &ref_message);
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
#endif
