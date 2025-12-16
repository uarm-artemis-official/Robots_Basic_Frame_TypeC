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
#include <span>
#include <type_traits>
#include <utility>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "comm_protocol_interface.hpp"
#include "message_center.hpp"
#include "quantize.hpp"
#include "string.h"
#include "subsystems_modules.hpp"
#include "topics.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"
#include "uart_isr.hpp"

namespace CommApp {

    inline namespace v1 {
        CommApp::CommApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                         IDebug& debug_ref, MW_CAN::ICAN& _can, Config _config,
                         isr::can::CAN_ISR& _can_isr)
            : RTOSApp(_rtos),
              mc(mc_ref),
              debug(debug_ref),
              can(_can),
              can_isr(_can_isr),
              config(_config) {}

        void CommApp::init() {
            board_status = debug.get_board_status();

            ASSERT(can_isr.register_routine(
                       isr::can::ECallbacks::MESSAGE_PENDING,
                       [this](MW_CAN::BUS bus, isr::can::CANFrame frame) {
                           can_isr_on_message_pending(bus, frame);
                       }),
                   "Failed to register CAN ISR routine.");
            ASSERT(can_isr.register_init(
                       [this](MW_CAN::ICAN&) { return can_isr_init(can); }),
                   "Failed to register CAN ISR init function.");
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
                    mc2::get_comm_id<mc2::GimbalCommand,
                                     mc2::RobotMC::Topics>();
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
                        std::memcpy(&quantized_pitch,
                                    &(comm_in.bytes.data()[2]),
                                    sizeof(int16_t));
                        std::memcpy(&(gimbal_command.command_bits),
                                    &(comm_in.bytes.data()[4]),
                                    sizeof(uint32_t));

                        gimbal_command.yaw = inv_quantize_float(
                            quantized_yaw, std::numeric_limits<int16_t>::min(),
                            std::numeric_limits<int16_t>::max(), -PI, PI);
                        gimbal_command.pitch = inv_quantize_float(
                            quantized_pitch,
                            std::numeric_limits<int16_t>::min(),
                            std::numeric_limits<int16_t>::max(), -PI, PI);
                        mc.pub_message(gimbal_command);
                        break;
                    }
                    case shoot_command_topic_id: {
                        mc2::ShootCommand shoot_command;
                        std::memcpy(&(shoot_command.command_bits),
                                    comm_in.bytes.data(), sizeof(uint32_t));
                        std::memcpy(&(shoot_command.extra_bits),
                                    &(comm_in.bytes.data()[4]),
                                    sizeof(uint32_t));
                        mc.pub_message(shoot_command);
                        break;
                    }
                    default:
                        break;
                }
            }
        };

        bool CommApp::transmit_interboard_message(
            const uint32_t message_id, const uint8_t message_data[8]) {
            return can.send_data(MW_CAN::BUS::CAN_2, message_id, 0,
                                 message_data, 8);
        }

        bool CommApp::can_isr_init(MW_CAN::ICAN&) {
            return true;
        }

        void CommApp::can_isr_on_message_pending(MW_CAN::BUS bus,
                                                 isr::can::CANFrame frame) {
            mc2::CommIn comm_in;
            if (bus == MW_CAN::BUS::CAN_2) {
                comm_in.topic_name = frame.stdid;
                std::memcpy(comm_in.bytes.data(), frame.payload,
                            sizeof(uint8_t) * frame.payload_length);
                mc.pub_message_from_isr(comm_in);
            }
        }

    }  // namespace v1

    namespace v2 {
        template <typename T>
        void CommApp::InterboardOperator::operator()() {
            static_assert(
                std::is_trivially_copyable<T>(),
                "Interboard message types must be trivially copyable.");
            // Implementation for interboard operation on type T

            if (T::destination != current_node) {
                std::span<uint8_t> temp_span(temp_buffer, T::serialized_size);
                T message;
                mc.get_message(message);
                T::serialize(message, temp_span);

                comm::protocol::TopicMessageMeta meta;
                meta.source = static_cast<uint8_t>(current_node);
                meta.destination = static_cast<uint8_t>(T::destination);
                meta.message_id = mc2::get_comm_id<T, mc2::RobotMC::Topics>();
                meta.payload_length = T::serialized_size;

                if (T::destination == mc2::MessageNode::Chassis ||
                    T::destination == mc2::MessageNode::All) {
                    can_comm.queue_send_message(temp_span.data(), meta);
                }

                if (T::destination == mc2::MessageNode::Gimbal ||
                    T::destination == mc2::MessageNode::All) {
                    uart_comm.queue_send_message(temp_span.data(), meta);
                }
            }
        }

        // Future CommApp v2 implementation.
        CommApp::CommApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                         IDebug& debug_ref, comm::CANComm<>& can_comm_ref,
                         comm::UARTComm<>& uart_comm_ref,
                         isr::uart::UART_ISR& uart_isr_ref)
            : RTOSApp(_rtos),
              mc(mc2_ref),
              debug(debug_ref),
              can_comm(can_comm_ref),
              uart_comm(uart_comm_ref),
              uart_isr(uart_isr_ref),
              op(mc2::MessageNode::All, mc2_ref, can_comm_ref, uart_comm_ref) {}

        void CommApp::init() {
            deserialize_directory =
                mc2::generate_byte_deserializers<mc2::RobotMC::Topics>();
            byte_serializers =
                mc2::generate_byte_serializers<mc2::RobotMC::Topics>();
            interboard_topic_ids =
                mc2::generate_interboard_topic_ids<mc2::RobotMC::Topics>();

            board_status = debug.get_board_status();

            if (board_status == BoardStatus_t::GIMBAL_BOARD) {
                bool register_init_success =
                    uart_isr.register_init([&](MW_UART::IUART& uart_instance) {
                        return uart_comm.start_receive(uart_instance);
                    });
                bool register_rountine_success = uart_isr.register_routine(
                    isr::uart::ECallbacks::RECEIVE_COMPLETE,
                    [&](MW_UART::IUART& uart_instance,
                        MW_UART::Peripheral peripheral) {
                        uart_comm.on_receive_complete(uart_instance,
                                                      peripheral);
                    });
                ASSERT(register_init_success && register_rountine_success,
                       "Failed to register both UART ISR init and routnine "
                       "functions.");
                op.current_node = mc2::MessageNode::Gimbal;
            } else {
                op.current_node = mc2::MessageNode::Chassis;
            }
            can_comm.init();
        }

        void CommApp::queue_interboard_messages() {
            for (uint8_t id : interboard_topic_ids) {
                const mc2::TopicMeta& topic_meta = mc.get_topic_meta(id);
                std::span<std::byte> byte_message_span =
                    std::span(to_publish_buffer.begin(), topic_meta.item_size);

                size_t byte_message_size = 0;
                auto res = mc.get_byte_message(byte_message_span,
                                               byte_message_size, id);

                ASSERT(byte_message_size == topic_meta.item_size,
                       "Message size mismatch in interboard message queuing.");

                if (res.has_value()) {
                    comm::protocol::TopicMessageMeta meta;

                    mc2::MessageNode current_node;
                    if (board_status == BoardStatus_t::GIMBAL_BOARD) {
                        current_node = mc2::MessageNode::Gimbal;
                    } else {
                        current_node = mc2::MessageNode::Chassis;
                    }

                    meta.source = static_cast<uint8_t>(current_node);
                    meta.destination =
                        static_cast<uint8_t>(topic_meta.destination);
                    meta.message_id = id;
                    meta.payload_length = topic_meta.serialized_size;

                    size_t topic_index = mc2::get_index_from_topic_id(id);

                    std::span<std::byte> serialized_span(
                        to_publish_serialized_buffer.begin(),
                        topic_meta.serialized_size);
                    bool has_serialized = byte_serializers[topic_index](
                        serialized_span, byte_message_span);

                    if (has_serialized) {
                        if (meta.destination ==
                                static_cast<uint8_t>(
                                    mc2::MessageNode::Chassis) ||
                            meta.destination ==
                                static_cast<uint8_t>(mc2::MessageNode::All)) {
                            can_comm.queue_send_message(serialized_span, meta);
                        }

                        if (meta.destination == static_cast<uint8_t>(
                                                    mc2::MessageNode::Gimbal) ||
                            meta.destination ==
                                static_cast<uint8_t>(mc2::MessageNode::All)) {
                            uart_comm.queue_send_message(serialized_span, meta);
                        }
                    }
                }
            }
        }

        void CommApp::publish_new_mesage_from_buffer(
            const comm::protocol::TopicMessageMeta& meta) {
            std::byte dst_buffer[256];
            size_t index = meta.message_id - mc2::TOPIC_ID_OFFSET;
            bool success = deserialize_directory[index](
                std::span(dst_buffer), std::span(message_temp_buffer));
            if (success) {
                mc.pub_byte_message(dst_buffer, meta.message_id);
            } else {
                ASSERT(false, "Failed to deserialize interboard message.");
            }
        }

        void CommApp::loop() {
            queue_interboard_messages();

            if (board_status == BoardStatus_t::GIMBAL_BOARD) {

                for (int i = 0; i < 5; i++) {
                    comm::protocol::TopicMessageMeta meta;
                    bool has_new_message =
                        uart_comm.get_message(message_temp_buffer, meta);
                    if (has_new_message) {
                        // Forward to chassis.
                        if (meta.destination ==
                            static_cast<uint8_t>(mc2::MessageNode::Chassis)) {
                            can_comm.queue_send_message(message_temp_buffer,
                                                        meta);
                        }

                        // Publish to message center topic.
                        if (meta.destination ==
                                static_cast<uint8_t>(mc2::MessageNode::All) ||
                            meta.destination == static_cast<uint8_t>(
                                                    mc2::MessageNode::Gimbal)) {
                            publish_new_mesage_from_buffer(meta);
                        }
                    }
                }
            }

            if (board_status == BoardStatus_t::CHASSIS_BOARD ||
                board_status == BoardStatus_t::GIMBAL_BOARD) {
                for (int i = 0; i < 5; i++) {
                    comm::protocol::TopicMessageMeta meta;
                    bool has_new_message =
                        can_comm.get_message(message_temp_buffer, meta);
                    if (has_new_message) {
                        // Publish to message center topic.
                        if (meta.destination ==
                                static_cast<uint8_t>(mc2::MessageNode::All) ||
                            meta.destination ==
                                static_cast<uint8_t>(
                                    mc2::MessageNode::Chassis)) {
                            publish_new_mesage_from_buffer(meta);
                        }
                    }
                }
            }

            for (int i = 0; i < 5; i++) {
                can_comm.send_next_frame();
            }
        }
    }  // namespace v2
}  // namespace CommApp