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

#include "apps_classes.hpp"
#include "apps_types.hpp"
#include "crc.hpp"
#include "string.h"

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
RefereeApp::RefereeApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                       IEventCenter& evt_center, modules::debug::Debug _debug,
                       IRefUI& ref_ui, isr::uart::UART_ISR& _uart_isr)
    : RTOSApp(_rtos),
      mc(mc_ref),
      event_center(evt_center),
      debug(_debug),
      ref_ui(ref_ui),
      uart_isr(_uart_isr) {}

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

    uart_isr.register_routine(
        isr::uart::ECallbacks::RECEIVE_COMPLETE,
        [this](MW_UART::IUART& uart, MW_UART::Peripheral peripheral) {
            uart_isr_receive_complete(uart, peripheral);
        });

    uart_isr.register_routine(
        isr::uart::ECallbacks::ON_ERROR,
        [this](MW_UART::IUART& uart, MW_UART::Peripheral peripheral) {
            uart_isr_on_error(uart, peripheral);
        });

    uart_isr.register_init(
        [this](MW_UART::IUART& uart) { return uart_isr_init(uart); });
}

void RefereeApp::loop() {
    // Main loop for referee app
    // outlines:
    // Read data from referee system (from Topic)
    if (mc.get_message(ref.referee_in).has_value()) {
        // Process received data
        read_ref_data();
        non_recv_count = 0;

        mc2::RefereeOut ref_out;
        ref_out.robot_id = ref.robot_status_data.robot_id;
        ref_out.robot_level = ref.robot_status_data.robot_level;
        ref_out.shoot_barrel_cooling_rate =
            ref.robot_status_data.shooter_barrel_cooling_value;
        ref_out.chassis_power_limit = ref.robot_status_data.chassis_power_limit;
        mc.pub_message(ref_out);
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

bool RefereeApp::uart_isr_init(MW_UART::IUART& uart) {
    return uart.receive_data(MW_UART::Peripheral::UART1,
                             uart_referee_in.ref_bytes.data(),
                             uart_referee_in.ref_bytes.size());
}

void RefereeApp::uart_isr_receive_complete(MW_UART::IUART& uart,
                                           MW_UART::Peripheral peripheral) {
    if (peripheral == MW_UART::Peripheral::UART1) {
        mc.pub_message_from_isr(uart_referee_in);
        memset(uart_referee_in.ref_bytes.data(), 0,
               uart_referee_in.ref_bytes.size());
        ASSERT(uart.receive_data(MW_UART::Peripheral::UART1,
                                 uart_referee_in.ref_bytes.data(),
                                 uart_referee_in.ref_bytes.size()),
               "Failed to start another receive after receive complete.");
    }
}

void RefereeApp::uart_isr_on_error(MW_UART::IUART& uart,
                                   MW_UART::Peripheral peripheral) {
    if (peripheral == MW_UART::Peripheral::UART1) {
        uart.abort_receive(peripheral);
        // Clear buffer
        memset(uart_referee_in.ref_bytes.data(), 0,
               sizeof(uart_referee_in.ref_bytes));

        // Restart DMA
        bool restart_reiceve_res =
            uart.receive_data(peripheral, uart_referee_in.ref_bytes.data(),
                              sizeof(uart_referee_in.ref_bytes));
        ASSERT(
            restart_reiceve_res,
            "Failed to restart UART receive after error for referee frames.");
    }
}

void RefereeApp::read_ref_data() {
    // Read data from referee system
    if (ref.referee_in.ref_bytes.data() == NULL) {
        // frame is NULL, return
        return;
    }
    /* copy frame header */
    memcpy(&ref.header, ref.referee_in.ref_bytes.data(), HEADER_LEN);

    /* frame header CRC8 verification */
    // FIXME: We don't know if we still need crc8 verification. if not , probably just update the pointer
    if (ref.header.sof == SOF_ID) {
        if (Verify_CRC8_Check_Sum((unsigned char*) &(ref.header), HEADER_LEN) ==
            1) {

            /* successfully verified */
            // ref.ref_cmd_id = (uint16_t)((ref.ref_rx_frame[HEADER_LEN] << 8) | ref.ref_rx_frame[HEADER_LEN + 1]);

            ref.ref_cmd_id = *(
                uint16_t*) (ref.referee_in.ref_bytes.data() +
                            HEADER_LEN);  //point to the addr of the cmd id (rx_frame[6] << 8 | rx_frame[5])

            memcpy(
                ref.ref_data,
                ref.referee_in.ref_bytes.data() + HEADER_LEN + CMD_LEN,
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

            if (*(ref.referee_in.ref_bytes.data() + HEADER_LEN + CMD_LEN +
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
    mc2::ChassisCommand chassis_command;
    if (mc.peek_message(chassis_command).has_value()) {
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
