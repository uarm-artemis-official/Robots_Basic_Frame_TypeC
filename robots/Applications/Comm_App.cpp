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
#include <cstdint>
#include <cstring>
#include <limits>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "quantize.hpp"
#include "string.h"
#include "subsystems_modules.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

namespace CommApp {
    CommApp::CommApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                     IDebug& debug_ref, MW_CAN::ICAN& _can, Config _config)
        : RTOSApp(_rtos),
          mc(mc_ref),
          debug(debug_ref),
          can(_can),
          config(_config) {}

    void CommApp::init() {
        board_status = debug.get_board_status();
    }

    void CommApp::loop() {
        mc2::CommOut comm_out;
        auto comm_out_message_ts = mc.get_message(comm_out);
        if (comm_out_message_ts.has_value()) {
            switch (config.op_mode) {
                case OperationMode::Normal:
                    transmit_interboard_message(comm_out.topic_name,
                                                comm_out.bytes.data());
                    break;
                case OperationMode::Loopback: {
                    mc2::CommIn comm_in;
                    memcpy(comm_in.bytes.data(), comm_out.bytes.data(),
                           sizeof(comm_in.bytes));
                    mc.pub_message(comm_in);
                    break;
                }
                default:
                    ASSERT(false, "Unsupported Comm App operation mode.");
            }
        }

        mc2::CommIn comm_in;
        auto comm_in_message_ts = mc.get_message(comm_in);
        if (comm_in_message_ts.has_value()) {
            // Ensure that IDs are evaluated at compile-time.
            constexpr uint32_t referee_out_topic_id =
                mc2::get_comm_id<mc2::RefereeOut, mc2::RobotMC::Topics>();
            constexpr uint32_t gimbal_command_topic_id =
                mc2::get_comm_id<mc2::GimbalCommand, mc2::RobotMC::Topics>();
            constexpr uint32_t shoot_command_topic_id =
                mc2::get_comm_id<mc2::ShootCommand, mc2::RobotMC::Topics>();
            constexpr uint32_t gimbal_relative_angles_topic_id =
                mc2::get_comm_id<mc2::GimbalRelativeAngles,
                                 mc2::RobotMC::Topics>();

            switch (comm_in.topic_name) {
                case referee_out_topic_id:
                    // TODO: Implement
                    break;
                case gimbal_relative_angles_topic_id: {
                    mc2::GimbalRelativeAngles rel_angles;
                    memcpy(&rel_angles.yaw, comm_in.bytes.data(),
                           sizeof(float));
                    memcpy(&rel_angles.pitch, &comm_in.bytes.data()[4],
                           sizeof(float));
                    mc.pub_message(rel_angles);
                } break;
                case gimbal_command_topic_id: {
                    mc2::GimbalCommand gimbal_command;
                    int16_t quantized_yaw;
                    int16_t quantized_pitch;

                    std::memcpy(&quantized_yaw, comm_in.bytes.data(),
                                sizeof(int16_t));
                    std::memcpy(&quantized_pitch, &(comm_in.bytes.data()[2]),
                                sizeof(int16_t));
                    std::memcpy(&(gimbal_command.command_bits),
                                &(comm_in.bytes.data()[4]), sizeof(uint32_t));

                    gimbal_command.yaw = inv_quantize_float(
                        quantized_yaw, std::numeric_limits<int16_t>::min(),
                        std::numeric_limits<int16_t>::max(), -PI, PI);
                    gimbal_command.pitch = inv_quantize_float(
                        quantized_pitch, std::numeric_limits<int16_t>::min(),
                        std::numeric_limits<int16_t>::max(), -PI, PI);
                    mc.pub_message(gimbal_command);
                    break;
                }
                case shoot_command_topic_id: {
                    mc2::ShootCommand shoot_command;
                    std::memcpy(&(shoot_command.command_bits),
                                comm_in.bytes.data(), sizeof(uint32_t));
                    std::memcpy(&(shoot_command.extra_bits),
                                &(comm_in.bytes.data()[4]), sizeof(uint32_t));
                    mc.pub_message(shoot_command);
                    break;
                }
                default:
                    break;
            }
        }
    };

    bool CommApp::transmit_interboard_message(const uint32_t message_id,
                                              const uint8_t message_data[8]) {
        return can.send_data(MW_CAN::BUS::CAN_2, message_id, 0, message_data,
                             8);
    }
}  // namespace CommApp