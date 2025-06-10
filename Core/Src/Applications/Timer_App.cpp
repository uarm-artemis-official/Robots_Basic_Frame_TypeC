/*******************************************************************************
* @file           : Timer_App.c
* @brief          : A software timer task to register different periodical task.
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#include "Timer_App.h"
#include "apps_defines.h"
#include "apps_types.h"
#include "debug.h"
#include "message_center.h"
#include "motors.h"
#include "public_defines.h"
#include "uarm_lib.h"
#include "uarm_os.h"

/**
* @brief  Timer app used to update the CAN data
* 		  Also reserve for other real time tasks
* @param  Not used
* @retval None
*/

TimerApp::TimerApp(IMotors& system_motors_ref,
                   IMessageCenter& message_center_ref, IDebug& debug_ref)
    : system_motors(system_motors_ref),
      message_center(message_center_ref),
      debug(debug_ref) {
    memset(&motor_tx_message, 0, sizeof(MotorSetMessage_t));
}

void TimerApp::init() {
    BoardStatus_t status = debug.get_board_status();
    Motor_Config_t config;
    switch (status) {
        case CHASSIS_BOARD: {
#ifdef SWERVE_CHASSIS
#ifdef SWERVE_CALIBRATE
            config = SWERVE_ZERO;
#else
            config = SWERVE;
#endif
#else
            config = DJI_CHASSIS;
#endif
            break;
        }
        case GIMBAL_BOARD: {
            config = DJI_GIMBAL;
            break;
        }
        default:
            ASSERT(0, "Unsupported board status in Timer.");
    }
    system_motors.init(config);
}

void TimerApp::loop() {
    uint8_t received_new_message =
        message_center.get_message(MOTOR_SET, &motor_tx_message, 0);
    if (received_new_message == 1) {
        for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
            if (motor_tx_message.can_ids[i] != 0) {
                system_motors.set_motor_voltage(
                    motor_tx_message.can_ids[i],
                    motor_tx_message.motor_can_volts[i]);
            }
        }
    }

    // system_motors.request_feedback(swerve_ids.at(i));

#ifndef DISABLE_MOTOR_SEND
    system_motors.send_motor_voltage();
#endif
}