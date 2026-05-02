#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "message_center.hpp"
#include "middleware_interfaces.hpp"
#include "simple_comm.hpp"
#include "uarm_lib.hpp"

namespace comm {
    template <typename T>
    concept DataCommand =
        simple_comm::CommandMessage<T> && mc2::MessageTopic<T>;

    template <typename T>
    struct IsDataCommand : std::bool_constant<simple_comm::CommandMessage<T> &&
                                              mc2::MessageTopic<T>> {};

    template <typename T>
    struct IsCommandMessage
        : std::bool_constant<simple_comm::CommandMessage<T>> {};

    template <typename T>
    struct IsMessageTopic : std::bool_constant<mc2::MessageTopic<T>> {};

    using IndexableDeserializer =
        std::function<bool(std::span<std::byte>, std::span<const std::byte>)>;

    // TODO: Apply additional controls for sending messages to internal topics.
    template <typename MessageCenter, typename MessageList>
    class Communication {
       private:
        using CommandList =
            uarm_lib::typelist::concept_filter_t<IsCommandMessage, MessageList>;
        using TopicList =
            uarm_lib::typelist::concept_filter_t<IsMessageTopic, MessageList>;
        using DataCommandList =
            uarm_lib::typelist::concept_filter_t<IsDataCommand, CommandList>;

        MessageCenter& message_center;
        simple_comm::SimpleCommCodec& simple_comm_codec;
        MW_CAN::ICAN& can;
        MW_UART::IUART& uart;
        // Mapping assumes that command IDs are incremental, small, and all start at a fixed offset.
        // If any of the above assumptions are violated, this mapping strategy will need to be reworked.
        std::array<std::pair<uint8_t, IndexableDeserializer>,
                   std::tuple_size_v<DataCommandList>>
            deserializer_map;
        simple_comm::NodeID current_node_id;

       public:
        Communication(MessageCenter& msg_center,
                      simple_comm::SimpleCommCodec& simple_comm_codec_ref,
                      MW_CAN::ICAN& can_ref, MW_UART::IUART& uart_ref)
            : message_center(msg_center),
              simple_comm_codec(simple_comm_codec_ref),
              can(can_ref),
              uart(uart_ref),
              current_node_id(simple_comm::NodeID::UnknownNode) {}

        void set_node_id(simple_comm::NodeID node_id) {
            current_node_id = node_id;
        }

        bool init() {
            deserializer_map = uarm_lib::typelist::functor_map<DataCommandList>(
                [&]<typename T>(T) {
                    return std::pair<uint8_t, IndexableDeserializer> {
                        T::MESSAGE_ID, [](std::span<std::byte> dst,
                                          std::span<const std::byte> src) {
                            if (src.size() < T::SERIALIZED_SIZE ||
                                dst.size() < sizeof(T)) {
                                return false;
                            }
                            T msg;
                            bool success = T::deserialize_payload(
                                src.first<T::SERIALIZED_SIZE>(), msg);
                            if (success) {
                                std::memcpy(dst.data(), &msg, sizeof(T));
                            }
                            return success;
                        }};
                });
            return true;
        }

        void forward_message(simple_comm::SimpleMessage msg) {
            ASSERT(msg.destination != static_cast<uint8_t>(current_node_id),
                   "Attempted to forward message to current node.");
            if (msg.destination ==
                    static_cast<uint8_t>(simple_comm::NodeID::MiniPC) ||
                msg.destination ==
                    static_cast<uint8_t>(simple_comm::NodeID::All)) {
                switch (current_node_id) {
                    case simple_comm::NodeID::Gimbal: {
                        std::array<std::byte,
                                   simple_comm::UART_MAX_MESSAGE_SIZE>
                            uart_buffer;
                        size_t message_size = 0;
                        simple_comm_codec.to_uart_bytes(
                            msg,
                            std::span<std::byte>(uart_buffer.data(),
                                                 uart_buffer.size()),
                            message_size);

                        uart.send_data(MW_UART::Peripheral::UART1,
                                       reinterpret_cast<const uint8_t*>(
                                           uart_buffer.data()),
                                       uart_buffer.size(), 1);
                        break;
                    }
                    default:
                        ASSERT(false, "Invalid node ID for forwarding.");
                }
            } else if (msg.destination ==
                           static_cast<uint8_t>(simple_comm::NodeID::Chassis) ||
                       msg.destination ==
                           static_cast<uint8_t>(simple_comm::NodeID::All)) {
                MW_CAN::CANFrame can_frame;
                simple_comm_codec.to_can_message<MW_CAN::CANFrame>(msg,
                                                                   can_frame);
                can.send_data(
                    MW_CAN::BUS::CAN_2B, can_frame.sid, can_frame.eid,
                    reinterpret_cast<const uint8_t*>(can_frame.payload.data()),
                    can_frame.dlc);
            } else {
                ASSERT(false, "Invalid message destination.");
            }
        }

        void process_command_message(const simple_comm::SimpleMessage& msg) {
            (void) msg;
            // TODO: Implement.
        }

        void can_isr_message_pending(MW_CAN::BUS bus, MW_CAN::CANFrame frame) {
            if (bus == MW_CAN::BUS::CAN_2B) {
                simple_comm::SimpleMessage in_msg;

                bool d_ok = simple_comm_codec.from_can_message(frame, in_msg);
                if (d_ok) {
                    if (in_msg.destination !=
                        static_cast<uint8_t>(current_node_id)) {
                        forward_message(in_msg);
                    } else {
                        switch (in_msg.message_type) {
                            case simple_comm::MessageType::COMMAND: {
                                process_command_message(in_msg);
                                break;
                            }
                            case simple_comm::MessageType::DATA: {
                                // TODO: Deserialize payload.
                                std::array<std::byte, 20> byte_out_buffer;

                                for (size_t i = 0; i < deserializer_map.size();
                                     ++i) {
                                    if (deserializer_map[i].first ==
                                        in_msg.id) {
                                        const mc2::TopicMeta& topic_meta =
                                            message_center.get_topic_meta(
                                                in_msg.id);

                                        bool success =
                                            deserializer_map[i].second(
                                                std::span(byte_out_buffer)
                                                    .first(
                                                        topic_meta.item_size),
                                                std::span(in_msg.payload)
                                                    .first(
                                                        in_msg.payload_size));
                                        ASSERT(success,
                                               "Deserialization failed for "
                                               "message with ID %d");

                                        message_center
                                            .template pub_byte_message_from_isr(
                                                std::span(byte_out_buffer)
                                                    .first(
                                                        topic_meta.item_size),
                                                in_msg.id);
                                        break;
                                    }
                                }
                                break;
                            }
                            default:
                                ASSERT(false, "Invalid message type.");
                        }
                    }
                }
            }
        }

        template <typename T>
        std::optional<MW_RTOS::TickType> get_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0) {
            return message_center.template get_message<T>(message,
                                                          ticks_to_wait);
        }

        template <typename T>
        std::optional<MW_RTOS::TickType> peek_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0) {
            return message_center.template peek_message<T>(message,
                                                           ticks_to_wait);
        }

        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0) {
            return message_center.template pub_message<T>(message,
                                                          ticks_to_wait);
        }

        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message_from_isr(
            T& message, bool* will_context_switch = nullptr) {
            return message_center.template pub_message_from_isr<T>(
                message, will_context_switch);
        }

        template <simple_comm::CommandMessage T>
        bool transmit_external_message(T& message, simple_comm::NodeID source,
                                       simple_comm::NodeID destination) {
            simple_comm::SimpleMessage out_msg;
            out_msg.message_type = T::MESSAGE_TYPE;
            out_msg.source = static_cast<uint8_t>(source);
            out_msg.destination = static_cast<uint8_t>(destination);
            out_msg.id = T::MESSAGE_ID;
            out_msg.payload_size = T::SERIALIZED_SIZE;
            bool s_ok = T::serialize_payload(
                message, std::span(out_msg.payload)
                             .template first<T::SERIALIZED_SIZE>());
            if (s_ok) {
                MW_CAN::CANFrame message_frame;
                simple_comm_codec.to_can_message<MW_CAN::CANFrame>(
                    out_msg, message_frame);
                // TODO: Add CAN transmit method that accepts CANFrame directly.
                return can.send_data(MW_CAN::BUS::CAN_2B, message_frame.sid,
                                     message_frame.eid,
                                     reinterpret_cast<const uint8_t*>(
                                         message_frame.payload.data()),
                                     message_frame.dlc);
            } else {
                return false;
            }
        }
    };
}  // namespace comm

#endif