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
#include "debug.hpp"
#include "message_center.hpp"
#include "quantize.hpp"
#include "simple_comm.hpp"
#include "string.h"
#include "subsystems_modules.hpp"
#include "topics.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"
#include "uart_isr.hpp"

namespace CommApp {
    namespace v2 {

        // TODO: Remove after testing alternative enqueue implementation.
        // template <typename T>
        // void CommApp::InterboardOperator::operator()() {
        //     static_assert(
        //         std::is_trivially_copyable<T>(),
        //         "Interboard message types must be trivially copyable.");
        //     // Implementation for interboard operation on type T

        //     if (T::destination != current_node) {
        //         std::span<uint8_t> temp_span(temp_buffer, T::serialized_size);
        //         T message;
        //         mc.get_message(message);
        //         T::serialize(message, temp_span);

        //         comm::protocol::TopicMessageMeta meta;
        //         meta.source = static_cast<uint8_t>(current_node);
        //         meta.destination = static_cast<uint8_t>(T::destination);
        //         meta.message_id = mc2::get_comm_id<T, mc2::RobotMC::Topics>();
        //         meta.payload_length = T::serialized_size;

        //         if (T::destination == mc2::MessageNode::Chassis ||
        //             T::destination == mc2::MessageNode::All) {
        //             can_comm.queue_send_message(temp_span.data(), meta);
        //         }

        //         if (T::destination == mc2::MessageNode::Gimbal ||
        //             T::destination == mc2::MessageNode::All) {
        //             uart_comm.queue_send_message(temp_span.data(), meta);
        //         }
        //     }
        // }

        // Future CommApp v2 implementation.
        CommApp::CommApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                         modules::debug::Debug _debug,
                         comm::CANComm<>& can_comm_ref,
                         comm::UARTComm<>& uart_comm_ref,
                         isr::uart::UART_ISR& uart_isr_ref)
            : RTOSApp(_rtos),
              mc(mc2_ref),
              debug(_debug),
              can_comm(can_comm_ref),
              uart_comm(uart_comm_ref),
              uart_isr(uart_isr_ref) {}

        void CommApp::init() {
            deserialize_directory =
                mc2::generate_byte_deserializers<mc2::RobotMC::Topics>();
            byte_serializers =
                mc2::generate_byte_serializers<mc2::RobotMC::Topics>();
            interboard_topic_ids =
                mc2::generate_interboard_topic_ids<mc2::RobotMC::Topics>();

            board_status = debug.get_board_config();

            if (board_status == modules::debug::BoardConfig::GIMBAL) {
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
                    if (board_status == modules::debug::BoardConfig::GIMBAL) {
                        current_node = mc2::MessageNode::Gimbal;
                    } else {
                        current_node = mc2::MessageNode::Chassis;
                    }

                    meta.source = static_cast<uint8_t>(current_node);
                    meta.destination =
                        static_cast<uint8_t>(topic_meta.destination);
                    meta.message_id = id;
                    meta.payload_length = topic_meta.serialized_size;

                    std::span<std::byte> serialized_span(
                        to_publish_serialized_buffer.begin(),
                        topic_meta.serialized_size);
                    bool has_serialized = mc.serialize_by_topic_id(
                        id, serialized_span, byte_message_span);

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

            if (board_status == modules::debug::BoardConfig::GIMBAL) {

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

            if (board_status == modules::debug::BoardConfig::CHASSIS ||
                board_status == modules::debug::BoardConfig::GIMBAL) {
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

    namespace v3 {
        CommApp::CommApp(
            MW_RTOS::IRTOS& _rtos, isr::can::CAN_ISR& _can_isr,
            isr::uart::UART_ISR& _uart_isr,
            simple_comm::SimpleComm<MAX_SIMPLE_COMM_FX_FIFO_SIZE>& _simple_comm,
            MW_CAN::ICAN& _can, MW_UART::IUART& _uart, mc2::RobotMC& mc2_ref,
            modules::debug::Debug _debug)
            : RTOSApp(_rtos),
              can_isr(_can_isr),
              uart_isr(_uart_isr),
              simple_comm(_simple_comm),
              mc(mc2_ref),
              can(_can),
              uart(_uart),
              debug(_debug) {}

        bool CommApp::init() {
            modules::debug::BoardConfig board_status = debug.get_board_config();
            if (board_status == modules::debug::BoardConfig::GIMBAL) {
                current_node = mc2::MessageNode::Gimbal;
            } else if (board_status == modules::debug::BoardConfig::CHASSIS) {
                current_node = mc2::MessageNode::Chassis;
            } else {
                ASSERT(false, "Unknown board configuration.");
                return false;
            }

            bool can_routine_success = can_isr.register_routine(
                isr::can::ECallbacks::MESSAGE_PENDING,
                [&](MW_CAN::BUS bus, isr::can::CANFrame frame) {
                    if (bus == MW_CAN::BUS::CAN_2B) {
                        // TODO: Remove after changing CANFrame definition in ISR.
                        MW_CAN::CANFrame mw_frame;
                        mw_frame.sid = frame.stdid;
                        mw_frame.eid = frame.extid;
                        mw_frame.dlc = frame.payload_length;
                        mw_frame.is_extended_id = true;
                        for (size_t i = 0; i < frame.payload_length; ++i) {
                            mw_frame.payload[i] = std::byte {frame.payload[i]};
                        }
                        simple_comm.can_isr_message_pending(bus, mw_frame);
                    }
                });

            bool uart_init_success = true;
            bool uart_routine_success = true;
            if (board_status == modules::debug::BoardConfig::GIMBAL) {
                uart_init_success =
                    uart_isr.register_init([&](MW_UART::IUART& uart_instance) {
                        return simple_comm.uart_isr_init(uart_instance);
                    });

                uart_routine_success = uart_isr.register_routine(
                    isr::uart::ECallbacks::RECEIVE_COMPLETE,
                    [&](MW_UART::IUART& uart_instance,
                        MW_UART::Peripheral peripheral) {
                        simple_comm.uart_isr_receive_complete(uart_instance,
                                                              peripheral);
                    });
            }

            return can_routine_success && uart_init_success &&
                   uart_routine_success;
        }

        void CommApp::loop() {
            for (size_t i = 0; i < MAX_SIMPLE_COMM_FX_FIFO_SIZE; i++) {
                simple_comm::SimpleMessage msg;
                bool has_new_message = simple_comm.get_rx_message(msg);
                if (has_new_message) {
                    (void) messages_to_process_buffer.push(msg);
                } else {
                    break;
                }
            }

            enqueue_interboard_messages();

            for (size_t i = 0; i < INTERNAL_FIFO_SIZE; i++) {
                simple_comm::SimpleMessage msg;
                bool has_new_message = messages_to_process_buffer.pop(msg);

                if (has_new_message) {
                    mc2::MessageNode msg_destination =
                        static_cast<mc2::MessageNode>(msg.destination);
                    if (msg_destination == current_node ||
                        msg_destination == mc2::MessageNode::All) {
                        std::span<const std::byte> payload_span(
                            msg.payload.data(), msg.payload_size);
                        std::span<std::byte> dst_span(
                            deserialize_message_buffer.begin(),
                            mc.get_topic_meta(msg.topic_id).item_size);
                        bool has_deserialized = mc.deserialize_by_topic_id(
                            msg.topic_id, dst_span, payload_span);
                        if (has_deserialized) {
                            mc.pub_byte_message(dst_span, msg.topic_id);
                        } else {
                            // TODO: Add warning logging for deserialization failure.
                        }
                    } else {
                        switch (msg_destination) {
                            case mc2::MessageNode::Chassis:
                                [[fallthrough]];
                            case mc2::MessageNode::Gimbal: {
                                // send_message_via_can(msg);
                                break;
                            }
                            case mc2::MessageNode::MiniPC: {
                                if (current_node == mc2::MessageNode::Chassis) {
                                    send_message_via_uart(msg);
                                } else if (current_node ==
                                           mc2::MessageNode::Gimbal) {
                                    send_message_via_uart(msg);
                                } else {
                                    ASSERT(false,
                                           "Unsupported node for "
                                           "MiniPC forwarding.");
                                }
                                break;
                            }
                            default:
                                // TODO: Add warning logging for unsupported forwarding.
                                break;
                        }
                    }
                } else {
                    break;
                }
            }
        }

        bool CommApp::send_message_via_can(
            const simple_comm::SimpleMessage& msg) {
            MW_CAN::CANFrame frame;
            simple_comm.format_can_message(msg, frame);
            return can.send_data(
                MW_CAN::BUS::CAN_2B, frame.sid, frame.eid,
                reinterpret_cast<uint8_t*>(frame.payload.data()), frame.dlc);
        }

        bool CommApp::send_message_via_uart(
            const simple_comm::SimpleMessage& msg) {
            std::span<std::byte> out_buffer(uart_out_buffer);
            size_t uart_message_size = 0;
            simple_comm.format_uart_message(msg, out_buffer, uart_message_size);
            uart.send_data(MW_UART::Peripheral::UART1,
                           reinterpret_cast<uint8_t*>(out_buffer.data()),
                           static_cast<uint32_t>(uart_message_size), 1);
            return true;
        }

        void CommApp::enqueue_interboard_messages() {
            auto interboard_topic_ids =
                mc2::generate_interboard_topic_ids<mc2::RobotMC::Topics>();
            for (uint8_t id : interboard_topic_ids) {
                const mc2::TopicMeta& topic_meta = mc.get_topic_meta(id);
                if (topic_meta.destination !=
                    static_cast<uint8_t>(current_node)) {
                    std::span<std::byte> byte_message_span = std::span(
                        to_publish_buffer.begin(), topic_meta.item_size);

                    size_t byte_message_size = 0;
                    auto res = mc.get_byte_message(byte_message_span,
                                                   byte_message_size, id);

                    if (res.has_value()) {
                        ASSERT(byte_message_size == topic_meta.item_size,
                               "Message size mismatch in interboard message "
                               "queuing.");

                        simple_comm::SimpleMessage msg;

                        msg.source = static_cast<uint8_t>(current_node);
                        msg.destination =
                            static_cast<uint8_t>(topic_meta.destination);
                        msg.topic_id = id;
                        msg.payload_size = topic_meta.serialized_size;
                        msg.payload.fill(std::byte {0});

                        std::span<std::byte> serialized_span(
                            msg.payload.begin(), topic_meta.serialized_size);
                        bool has_serialized = mc.serialize_by_topic_id(
                            id, serialized_span, byte_message_span);

                        if (has_serialized) {
                            (void) messages_to_process_buffer.push(msg);
                        } else {
                            ASSERT(false,
                                   "Failed to serialize interboard message.");
                        }
                    }
                }
            }
        }
    }  // namespace v3
}  // namespace CommApp