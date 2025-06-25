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
#include <cstring>
#include <limits>
#include "apps_defines.h"
#include "quantize.hpp"
#include "string.h"
#include "uarm_lib.h"
#include "uarm_math.h"
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
            case REFEREE_OUT:
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
            case COMMAND_GIMBAL: {
                GimbalCommandMessage_t gimbal_command;
                int16_t quantized_yaw;
                int16_t quantized_pitch;

                std::memcpy(&quantized_yaw, incoming_message.data,
                            sizeof(int16_t));
                std::memcpy(&quantized_pitch, &(incoming_message.data[2]),
                            sizeof(int16_t));
                std::memcpy(&(gimbal_command.command_bits),
                            &(incoming_message.data[4]), sizeof(uint32_t));

                gimbal_command.yaw = inv_quantize_float(
                    quantized_yaw, std::numeric_limits<int16_t>::min(),
                    std::numeric_limits<int16_t>::max(), -PI, PI);
                gimbal_command.pitch = inv_quantize_float(
                    quantized_pitch, std::numeric_limits<int16_t>::min(),
                    std::numeric_limits<int16_t>::max(), -PI, PI);

                message_center.pub_message(COMMAND_GIMBAL, &gimbal_command);
                break;
            }
            case COMMAND_SHOOT: {
                ShootCommandMessage_t shoot_command;
                std::memcpy(&(shoot_command.command_bits),
                            incoming_message.data, sizeof(uint32_t));
                std::memcpy(&(shoot_command.extra_bits),
                            &(incoming_message.data[4]), sizeof(uint32_t));

                message_center.pub_message(COMMAND_SHOOT, &shoot_command);
                break;
            }
            default:
                break;
        }
    }
};