/*******************************************************************************
* @file           : PC_UART_App.c
* @brief          : Upper computer communication task
* @restructed     : Mar, 2024
* @maintainer     : James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include "PC_UART_App.h"
#include <cstring>
#include "apps_defines.h"
#include "pack_handler.h"  // TODO: Remove
#include "pc_comm.h"
#include "uarm_lib.h"
#include "uarm_os.h"

PCUARTApp::PCUARTApp(IMessageCenter& message_center_ref, IMotors& motors_)
    : message_center(message_center_ref), motors(motors_) {
    memset(new_pack_buffer, 0, sizeof(uint8_t) * 64);
}

void PCUARTApp::init() {
    start_receive(new_pack_buffer);
}

void PCUARTApp::loop() {
    // UC_Board_Data_Pack_t send_pack;
    // pack_init(&send_pack, get_data_size(UC_BOARD_DATA_HEADER));
    // send_pack.robot_color = 2;
    // send_pack.pitch = 1.111f;
    // send_pack.yaw = 2.222f;
    // send_pack.accel_x = 0.999f;
    // send_pack.accel_y = 0.888f;
    // send_pack.wheel_rpm[0] = 0.1f;
    // send_pack.wheel_rpm[1] = 0.2f;
    // send_pack.wheel_rpm[2] = 0.3f;
    // send_pack.wheel_rpm[3] = 0.4f;

    if (idle_count == 200) {
        restart_receive(new_pack_buffer);
        idle_count = 0;
    }

    while (message_center.get_message(UC_PACK_IN, new_pack_buffer, 0) ==
           pdTRUE) {
        if (PCComm::uc_check_pack_integrity(new_pack_buffer,
                                            MAX_PACK_BUFFER_SIZE) == 0) {
            switch (new_pack_buffer[0]) {
                case UC_AUTO_AIM_HEADER: {
                    UC_Auto_Aim_Pack_t aim_pack;
                    std::memcpy(&aim_pack, new_pack_buffer + PACK_HEADER_SIZE,
                                get_data_size(new_pack_buffer[0]));
                    if (aim_pack.target_num > 0) {
                        float deltas[] = {aim_pack.delta_yaw,
                                          aim_pack.delta_pitch};
                        recent_deltas[0] = aim_pack.delta_yaw;
                        recent_deltas[1] = aim_pack.delta_pitch;
                        message_center.pub_message(AUTO_AIM, deltas);
                    }
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
    while (message_center.get_message(UC_PACK_OUT, new_send_buffer, 0) ==
           pdTRUE) {
        PCComm::send_bytes(new_send_buffer, 196);
    }
#else
    send_swerve_data();
#endif
}

void PCUARTApp::send_swerve_data() {
#ifdef SWERVE_CHASSIS
    MotorReadMessage_t read_message;
    std::array<uint32_t, 4> steer_motor_ids = {
        SWERVE_STEER_MOTOR1,
        SWERVE_STEER_MOTOR2,
        SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4,
    };
    std::array<uint32_t, 4> drive_motor_ids = {
        CHASSIS_WHEEL1,
        CHASSIS_WHEEL2,
        CHASSIS_WHEEL3,
        CHASSIS_WHEEL4,
    };
    std::array<LK_Motor_Torque_Feedback_t, 4> steer_feedback;
    std::array<Motor_Feedback_t, 4> drive_feedback;

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
            for (size_t j = 0; j < steer_motor_ids.size(); j++) {
                if (steer_motor_ids.at(j) == read_message.can_ids[i]) {
                    motors.get_raw_feedback(steer_motor_ids.at(j),
                                            read_message.feedback[i],
                                            &(steer_feedback.at(j)));
                    break;
                }
            }

            for (size_t j = 0; j < drive_motor_ids.size(); j++) {
                if (drive_motor_ids.at(j) == read_message.can_ids[i]) {
                    motors.get_raw_feedback(drive_motor_ids.at(j),
                                            read_message.feedback[i],
                                            &(drive_feedback.at(j)));
                    break;
                }
            }
        }
    }
#endif
}