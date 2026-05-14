/*******************************************************************************
* @file           : PC_UART_App.c
* @brief          : Upper computer communication task
* @restructed     : Mar, 2024
* @maintainer     : James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include <cstring>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

PCUARTApp::PCUARTApp(
    MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication_ref,
    IMotors& motors_, IPCComm& pc_comm_, isr::uart::UART_ISR& _uart_isr)
    : RTOSApp(_rtos),
      mc(mc_ref),
      communication(communication_ref),
      motors(motors_),
      pc_comm(pc_comm_),
      uart_isr(_uart_isr) {
    memset(uc_pack_in.bytes.data(), 0, sizeof(uc_pack_in.bytes));
}

void PCUARTApp::init() {
    pc_comm.start_receive(uc_pack_in.bytes.data());

    ASSERT(uart_isr.register_routine(
               isr::uart::ECallbacks::RECEIVE_COMPLETE,
               [this](MW_UART::IUART& uart, MW_UART::Peripheral peripheral) {
                   uart_isr_receive_complete(uart, peripheral);
               }),
           "Failed to register UART ISR receive complete routine.");
    ASSERT(uart_isr.register_init(
               [this](MW_UART::IUART& uart) { return uart_isr_init(uart); }),
           "Failed to register UART ISR init function.");
}

void PCUARTApp::loop() {
    if (idle_count == 5000) {
        pc_comm.restart_receive(uc_pack_in.bytes.data());
        idle_count = 0;
    }

    while (mc.get_message(uc_pack_in).has_value()) {
        if (pc_comm.uc_check_pack_integrity(uc_pack_in.bytes.data(),
                                            MAX_PACK_BUFFER_SIZE) == 0) {
            switch (uc_pack_in.bytes[0]) {
                case UC_AUTO_AIM_HEADER: {
                    // TODO: Double check validity of pack and AutoAim struct.
                    mc2::AutoAim auto_aim;
                    std::memcpy(&auto_aim,
                                uc_pack_in.bytes.data() + PACK_HEADER_SIZE,
                                pc_comm.get_data_size(uc_pack_in.bytes[0]));
                    if (auto_aim.target_num > 0) {
                        auto_aim.delta_yaw = value_limit(
                            auto_aim.delta_yaw, degrees_to_radians(-15.f),
                            degrees_to_radians(15.f));
                        recent_deltas[0] = auto_aim.delta_yaw;
                        recent_deltas[1] = auto_aim.delta_pitch;
                        mc.pub_message(auto_aim);
                    }

                    mc2::ShootCommand shoot_command;
                    shoot_command.extra_bits = 0;
                    if (auto_aim.should_shoot) {
                        shoot_command.command_bits =
                            static_cast<uint8_t>(SHOOT_CONT);
                    } else {
                        shoot_command.command_bits =
                            static_cast<uint8_t>(SHOOT_CEASE);
                    }
                    mc.pub_message(shoot_command);
                    break;
                }
                case UC_FLOW_CONTROL_HEADER: {
                    //						memcpy(&uc_flow_control_pack, new_pack_buffer, UC_FLOW_CONTROL_DATA_SIZE);
                    //						process_flow_control();
                    break;
                }
                default:
                    break;
            }
        }
        idle_count = 0;
    }
    idle_count = (idle_count + 1) % 100000;

    // IMU Transmission
    // TODO: Send chassis x and y acceleration, robot color, and wheel RPMs.
    // This requires enabling IMU task for chassis and modifying Comm task.
    // send_pack.pitch += 0.1;
    // uc_send_board_data(&send_pack);

#ifndef SWERVE_CHASSIS
    // while (message_center.get_message(UC_PACK_OUT, new_send_buffer, 0) ==
    //        pdTRUE) {
    //     pc_comm.send_bytes(new_send_buffer, 196);
    // }
#else
    send_swerve_data();
#endif
}

bool PCUARTApp::uart_isr_init(MW_UART::IUART& uart) {
    return uart.receive_data(MW_UART::Peripheral::UART_1,
                             uart_pack_in.bytes.data(),
                             uart_pack_in.bytes.size());
}

void PCUARTApp::uart_isr_receive_complete(MW_UART::IUART& uart,
                                          MW_UART::Peripheral peripheral) {
    if (peripheral == MW_UART::Peripheral::UART_1) {
        mc.pub_message_from_isr(uart_pack_in);
        ASSERT(uart.receive_data(MW_UART::Peripheral::UART_1,
                                 uart_pack_in.bytes.data(),
                                 uart_pack_in.bytes.size()),
               "Failed to start another UART receive after receive complete.");
    }
}

void PCUARTApp::send_swerve_data() {
    // #ifdef SWERVE_CHASSIS
    //     MotorReadMessage_t read_message;
    //     std::array<uint32_t, 4> steer_motor_ids = {
    //         SWERVE_STEER_MOTOR1,
    //         SWERVE_STEER_MOTOR2,
    //         SWERVE_STEER_MOTOR3,
    //         SWERVE_STEER_MOTOR4,
    //     };
    //     std::array<uint32_t, 4> drive_motor_ids = {
    //         CHASSIS_WHEEL1,
    //         CHASSIS_WHEEL2,
    //         CHASSIS_WHEEL3,
    //         CHASSIS_WHEEL4,
    //     };
    //     std::array<LK_Motor_Torque_Feedback_t, 4> steer_feedback;
    //     std::array<Motor_Feedback_t, 4> drive_feedback;

    //     uint8_t new_read_message =
    //         message_center.peek_message(MOTOR_READ, &read_message, 0);
    //     if (new_read_message == 1) {
    //         for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
    //             for (size_t j = 0; j < steer_motor_ids.size(); j++) {
    //                 if (steer_motor_ids.at(j) == read_message.can_ids[i]) {
    //                     motors.get_raw_feedback(steer_motor_ids.at(j),
    //                                             read_message.feedback[i],
    //                                             &(steer_feedback.at(j)));
    //                     break;
    //                 }
    //             }

    //             for (size_t j = 0; j < drive_motor_ids.size(); j++) {
    //                 if (drive_motor_ids.at(j) == read_message.can_ids[i]) {
    //                     motors.get_raw_feedback(drive_motor_ids.at(j),
    //                                             read_message.feedback[i],
    //                                             &(drive_feedback.at(j)));
    //                     break;
    //                 }
    //             }
    //         }
    //     }
    // #endif
}