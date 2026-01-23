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

#include <cstring>
#include "ISRs/can_isr.hpp"
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "uarm_lib.hpp"

/**
* @brief  Timer app used to update the CAN data
* 		  Also reserve for other real time tasks
* @param  Not used
* @retval None
*/

TimerApp::TimerApp(MW_RTOS::IRTOS& _rtos, IMotors& system_motors_ref,
                                     mc2::RobotMC& mc2_ref, modules::debug::Debug& _debug,
                                     isr::can::CAN_ISR& _can_isr)
    : RTOSApp(_rtos),
      system_motors(system_motors_ref),
      mc(mc2_ref),
      debug(_debug),
      can_isr(_can_isr) {}

void TimerApp::init() {
    modules::debug::BoardConfig status = debug.get_board_config();
    Motor_Config_t config;
    switch (status) {
        case modules::debug::BoardConfig::CHASSIS: {
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
        case modules::debug::BoardConfig::GIMBAL: {
            config = DJI_GIMBAL;
            break;
        }
        default:
            ASSERT(0, "Unsupported board status in Timer.");
    }
    system_motors.init(config);

    ASSERT(can_isr.register_routine(
               isr::can::ECallbacks::MESSAGE_PENDING,
               [this](MW_CAN::BUS bus, isr::can::CANFrame frame) {
                   can_isr_message_receive(bus, frame);
               }),
           "Failed to register CAN ISR routine.");
    ASSERT(can_isr.register_init(
               [this](MW_CAN::ICAN& can) { return can_isr_init(can); }),
           "Failed to register CAN ISR init function.");
}

void TimerApp::loop() {
    auto message_ts = mc.get_message(motor_set);
    if (message_ts.has_value()) {
        for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
            if (motor_set.can_ids[i] != 0) {
                system_motors.set_motor_voltage(motor_set.can_ids[i],
                                                motor_set.motor_can_volts[i]);
            }
        }
    }

    // system_motors.request_feedback(swerve_ids.at(i));

#ifndef DISABLE_MOTOR_SEND
    system_motors.send_motor_voltage();
#endif
}

void TimerApp::parse_motor_feedback(isr::can::CANFrame frame) {
    uint8_t free_index = 0xff;
    for (int i = 0; i < 8; i++) {
        if (motor_read.can_ids[i] == 0)
            free_index = i;
        if (motor_read.can_ids[i] == frame.stdid) {
            free_index = i;
            break;
        }
    }

    if (free_index != 0xff && free_index < MAX_MOTOR_COUNT) {
        std::memcpy(motor_read.feedback[free_index], frame.payload,
                    sizeof(frame.payload_length));
        motor_read.can_ids[free_index] =
            static_cast<Motor_CAN_ID_t>(frame.stdid);
        mc.pub_message_from_isr(motor_read);
    }
}

void TimerApp::can_isr_message_receive(MW_CAN::BUS bus,
                                       isr::can::CANFrame frame) {
    if (bus == MW_CAN::BUS::CAN_1) {
        parse_motor_feedback(frame);
    } else if (bus == MW_CAN::BUS::CAN_2 &&
               can_isr_config == CANISRConfig::SentryChassis) {
        if (SWERVE_STEER_MOTOR1 <= frame.stdid &&
            frame.stdid <= SWERVE_STEER_MOTOR4) {
            parse_motor_feedback(frame);
        }
    }
}

bool TimerApp::can_isr_init(MW_CAN::ICAN&) {
    std::memset(&motor_read, 0, sizeof(motor_read));

    modules::debug::BoardConfig status = debug.get_board_config();
    switch (status) {
        case modules::debug::BoardConfig::CHASSIS: {
#ifdef SWERVE_CHASSIS
            can_isr_config = CANISRConfig::SentryChassis;
#else
            can_isr_config = CANISRConfig::OtherRobot;
#endif
            break;
        }
        case modules::debug::BoardConfig::GIMBAL: {
            can_isr_config = CANISRConfig::OtherRobot;
            break;
        }
        default:
            ASSERT(0, "Unsupported board status in Timer.");
    }

    return true;
}