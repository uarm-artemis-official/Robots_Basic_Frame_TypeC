#ifndef __SIMPLE_COMM_UTILS_HPP
#define __SIMPLE_COMM_UTILS_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <tuple>
#include "message_center.hpp"

namespace simple_comm {
    // Single data message constants and types.
    constexpr std::byte DATA_MAGIC_TRIBIT = std::byte {0x1};

    // Command message constants and types.
    constexpr std::byte COMMAND_MAGIC_TRIBIT = std::byte {0x3};
    constexpr uint32_t ACK_TIMEOUT_MS = 500;
    constexpr size_t MAX_RETRIES = 3;

    template <typename T>
    concept CommandMessage =
        std::is_trivially_copyable_v<T> &&
        requires(T msg, std::span<std::byte, T::SERIALIZED_SIZE> dst,
                 std::span<const std::byte, T::SERIALIZED_SIZE> src) {
        requires std::same_as<decltype(T::SERIALIZED_SIZE), const size_t>;
        requires std::same_as<decltype(T::ID), const uint8_t>;
        {T::serialize_payload(msg, dst)}->std::same_as<bool>;
        {T::deserialize_payload(src, msg)}->std::same_as<bool>;
    };

    template <std::byte MAGIC_, size_t SERIALIZED_SIZE_, uint8_t ID_>
    struct MessageMeta {
        static constexpr std::byte MAGIC = MAGIC_;
        static constexpr size_t SERIALIZED_SIZE = SERIALIZED_SIZE_;
        static constexpr uint8_t ID = ID_;
    };

    // General protocol constants and types.
    constexpr size_t MAX_PAYLOAD_SIZE = 8;

    constexpr size_t UART_HEADER_SIZE = 4;
    constexpr size_t UART_TRAILER_SIZE = 2;
    constexpr size_t UART_MAX_MESSAGE_SIZE =
        UART_HEADER_SIZE + MAX_PAYLOAD_SIZE + UART_TRAILER_SIZE;

    enum class MessageType {
        DATA = static_cast<uint8_t>(DATA_MAGIC_TRIBIT),
        COMMAND = static_cast<uint8_t>(COMMAND_MAGIC_TRIBIT),
    };

    struct SimpleMessage {
        MessageType message_type;
        uint8_t source;
        uint8_t destination;
        uint8_t id;
        uint8_t payload_size;
        std::array<std::byte, MAX_PAYLOAD_SIZE> payload;
    };

    namespace utils {
        using IndexableDeserializer = std::function<bool(
            std::span<std::byte>, std::span<const std::byte>)>;
        using IndexableSerializer = std::function<bool(
            std::span<std::byte>, std::span<const std::byte>)>;

        template <typename Registry, size_t index>
        consteval auto generate_message_deserializers_impl() {
            using T = std::tuple_element_t<index, Registry>;
            return [&](std::span<std::byte> dst,
                       std::span<const std::byte> src) {
                if constexpr (mc2::InterboardMessageTopic<T>) {
                    T msg;
                    bool success =
                        T::deserialize(msg, src.first<T::serialized_size>());
                    if (success) {
                        std::memcpy(dst.data(), &msg, sizeof(T));
                    }
                    return success;
                } else {
                    return false;
                }
            };
        }

        template <typename Registry>
        constexpr auto generate_message_deserializers() {
            constexpr size_t registry_size = std::tuple_size_v<Registry>;
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<IndexableDeserializer, registry_size> {
                    generate_message_deserializers_impl<Registry, Is>()...};
            }(std::make_index_sequence<registry_size> {});
        }

        template <typename Registry, size_t index>
        consteval auto generate_message_serializers_impl() {
            using T = std::tuple_element_t<index, Registry>;
            return
                [&](std::span<std::byte> dst, std::span<const std::byte> src) {
                    if constexpr (mc2::InterboardMessageTopic<T>) {
                        T msg;

                        if (src.size() < sizeof(T) ||
                            dst.size() < T::serialized_size) {
                            return false;
                        }

                        std::memcpy(&msg, src.data(), sizeof(T));

                        bool success =
                            T::serialize(msg, dst.first<T::serialized_size>());
                        return success;
                    } else {
                        return false;
                    }
                };
        }

        template <typename Registry>
        constexpr auto generate_message_serializers() {
            constexpr size_t registry_size = std::tuple_size_v<Registry>;
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<IndexableSerializer, registry_size> {
                    generate_message_serializers_impl<Registry, Is>()...};
            }(std::make_index_sequence<registry_size> {});
        }

        template <typename TopicRegistry>
        consteval auto generate_interboard_message_ids() {
            constexpr size_t registry_size = std::tuple_size_v<TopicRegistry>;

            constexpr auto collect_interboard_indices = []<size_t... Is>(
                                                            std::index_sequence<
                                                                Is...>) {
                constexpr size_t MaxInterboardTopics = sizeof...(Is);
                std::array<size_t, MaxInterboardTopics> indices = {};
                size_t count = 0;

                (
                    [&] {
                        using TopicType =
                            std::tuple_element_t<Is, TopicRegistry>;
                        if constexpr (mc2::InterboardMessageTopic<TopicType>) {
                            indices[count++] = Is;
                            static_assert(TopicType::serialized_size <=
                                              simple_comm::MAX_PAYLOAD_SIZE,
                                          "Interboard topic serialized_size "
                                          "exceeds simple_comm payload limit");
                        }
                    }(),
                    ...);

                return std::make_pair(indices, count);
            }(std::make_index_sequence<registry_size> {});

            constexpr size_t interboard_count =
                collect_interboard_indices.second;
            std::array<uint8_t, interboard_count> ids = {};
            for (size_t i = 0; i < interboard_count; ++i) {
                size_t topic_index = collect_interboard_indices.first[i];
                ids[i] = mc2::get_topic_id_from_index(topic_index);
            }

            return ids;
        }
    }  // namespace utils

    // Template codec that serializes/deserializes SimpleMessage to/from
    // a user-provided CAN POD type (CANPod). CANPod must be a POD with
    // members: uint32_t eid; uint16_t sid; bool is_extended_id; uint8_t dlc; std::array<std::byte,8> payload;
    class SimpleCommCodec {
       public:
        static bool is_recognized_magic(uint8_t magic) {
            return magic == static_cast<uint8_t>(DATA_MAGIC_TRIBIT) ||
                   magic == static_cast<uint8_t>(COMMAND_MAGIC_TRIBIT);
        }

        /**
             * @brief Serialize a SimpleMessage into a CAN POD frame.
             *
             * @pre The message payload must fit into a single CAN frame (maximum 8 bytes for CAN2.0B),
             *      so it can be represented by the CAN frame DLC.
             * @pre The `destination` field must be representable in 4 bits because it is packed into
             *      the extended identifier. Providing a larger value will trigger an assertion.
             * @pre The `source` field must be representable in 4 bits because it is packed into
             *      the extended identifier. Providing a larger value will trigger an assertion.
             *
             * @param msg The logical message to serialize.
             * @param out_frame Output CAN POD to populate.
             */
        template <typename CANPod>
        static void to_can_message(const SimpleMessage& msg,
                                   CANPod& out_frame) {
            ASSERT(msg.payload_size <= MAX_PAYLOAD_SIZE,
                   "Payload size exceeds maximum allowed.");
            ASSERT(msg.destination <= 0x0F,
                   "Destination ID exceeds 4-bit limit.");
            ASSERT(msg.source <= 0x0F, "Source ID exceeds 4-bit limit.");

            uint32_t eid = 0;
            eid |= (static_cast<uint32_t>(
                        static_cast<uint8_t>(msg.message_type) & 0x07)
                    << 26);
            eid |= (static_cast<uint32_t>(msg.destination & 0x0F) << 22);
            eid |= (static_cast<uint32_t>(msg.source & 0x0F) << 18);
            eid |= (static_cast<uint32_t>(msg.id) << 10);

            out_frame.eid = eid;
            out_frame.sid = 0;
            out_frame.is_extended_id = true;
            out_frame.dlc = msg.payload_size;
            std::copy_n(msg.payload.begin(), msg.payload_size,
                        out_frame.payload.begin());
        }

        /**
             * @brief Deserialize a CAN POD frame into a SimpleMessage.
             *
             * This performs wire validation (magic tribit and payload size).
             * Returns false on invalid / non-SimpleComm frames.
             *
             * @param frame The CAN POD received from the bus.
             * @param out_msg Destination to populate on success.
             * @return true if deserialization succeeded and the frame is a SimpleComm message.
             */
        template <typename CANPod>
        static bool from_can_message(const CANPod& frame,
                                     SimpleMessage& out_msg) {
            uint8_t magic = static_cast<uint8_t>((frame.eid >> 26) & 0x07);
            if (!is_recognized_magic(magic)) {
                return false;
            }

            out_msg.message_type = static_cast<MessageType>(magic);
            out_msg.destination =
                static_cast<uint8_t>((frame.eid >> 22) & 0x0F);
            out_msg.source = static_cast<uint8_t>((frame.eid >> 18) & 0x0F);
            out_msg.id = static_cast<uint8_t>((frame.eid >> 10) & 0xFF);
            out_msg.payload_size = frame.dlc;
            if (out_msg.payload_size > MAX_PAYLOAD_SIZE) {
                return false;
            }
            std::copy(frame.payload.begin(), frame.payload.begin() + frame.dlc,
                      out_msg.payload.begin());
            return true;
        }

        /**
             * @brief Calculate the UART checksum for a header+payload buffer.
             *
             * The checksum is a 16-bit sum (LSB first) over the header and payload
             * bytes (does not include the 2-byte trailer).
             *
             * @param buf Pointer to the bytes to sum.
             * @param len Number of bytes to include in the sum.
             * @return 16-bit checksum value.
             */
        static uint16_t calc_uart_checksum(const std::byte* buf, size_t len) {
            uint16_t checksum = 0;
            for (size_t i = 0; i < len; ++i) {
                checksum += static_cast<uint8_t>(buf[i]);
            }
            return checksum;
        }

        /**
             * @brief Serialize a SimpleMessage to UART wire bytes.
             *
             * @pre The message payload must be small enough to be sent as a single UART packet
             *      (maximum 8 bytes — this matches the CAN2.0B single-frame payload limit used by the protocol).
             * @pre The `destination` and `source` identifiers must fit into 4 bits each because
             *      they are compacted into a single header byte.
             * @pre The provided `out_buffer` must be large enough to hold the worst-case packet
             *      (header + maximum payload + 2-byte checksum). If it is smaller, the function
             *      will assert rather than silently overflow the buffer.
             *
             * @param msg Message to serialize.
             * @param out_buffer Output buffer to write full UART message into.
             * @param out_length Set to the number of bytes written on success.
             */
        static void to_uart_bytes(const SimpleMessage& msg,
                                  std::span<std::byte> out_buffer,
                                  size_t& out_length) {
            ASSERT(msg.payload_size <= MAX_PAYLOAD_SIZE,
                   "Payload size exceeds maximum allowed.");
            ASSERT(msg.destination <= 0x0F,
                   "Destination ID exceeds 4-bit limit.");
            ASSERT(msg.source <= 0x0F, "Source ID exceeds 4-bit limit.");
            ASSERT(out_buffer.size() >= UART_MAX_MESSAGE_SIZE,
                   "Output buffer too small for UART message.");

            out_buffer[0] = static_cast<std::byte>(msg.message_type);
            out_buffer[1] = std::byte {static_cast<uint8_t>(
                (msg.destination << 4) | (msg.source & 0x0F))};
            out_buffer[2] = std::byte {msg.id};
            out_buffer[3] = std::byte {msg.payload_size};
            std::copy_n(msg.payload.begin(), msg.payload_size,
                        out_buffer.begin() + UART_HEADER_SIZE);

            uint16_t checksum = calc_uart_checksum(
                out_buffer.data(), UART_HEADER_SIZE + msg.payload_size);

            out_buffer[UART_HEADER_SIZE + msg.payload_size] =
                std::byte {static_cast<uint8_t>(checksum & 0xFF)};
            out_buffer[UART_HEADER_SIZE + msg.payload_size + 1] =
                std::byte {static_cast<uint8_t>((checksum >> 8) & 0xFF)};

            out_length =
                UART_HEADER_SIZE + msg.payload_size + UART_TRAILER_SIZE;
        }

        /**
             * @brief Deserialize UART wire bytes into a SimpleMessage.
             *
             * Performs validation on magic tribit, payload size bounds, and checksum.
             * Returns false on any wire-format validation failure.
             *
             * @param in_buffer Input buffer containing the full UART message.
             * @param out_msg Destination SimpleMessage to populate.
             * @return true if deserialization succeeded and checksum/magic matched.
             */
        static bool from_uart_bytes(std::span<const std::byte> in_buffer,
                                    SimpleMessage& out_msg) {
            if (in_buffer.size() < UART_HEADER_SIZE + UART_TRAILER_SIZE) {
                return false;
            }
            if (!is_recognized_magic(static_cast<uint8_t>(in_buffer[0]))) {
                return false;
            }
            uint8_t payload_size = static_cast<uint8_t>(in_buffer[3]);
            if (payload_size == 0 || payload_size > MAX_PAYLOAD_SIZE) {
                return false;
            }
            size_t expected_len =
                UART_HEADER_SIZE + payload_size + UART_TRAILER_SIZE;
            if (in_buffer.size() < expected_len) {
                return false;
            }

            uint16_t calc_checksum = calc_uart_checksum(
                in_buffer.data(), UART_HEADER_SIZE + payload_size);
            uint16_t received_checksum =
                static_cast<uint16_t>(
                    in_buffer[UART_HEADER_SIZE + payload_size]) |
                (static_cast<uint16_t>(
                     in_buffer[UART_HEADER_SIZE + payload_size + 1])
                 << 8);

            if (calc_checksum != received_checksum) {
                return false;
            }

            out_msg.source = (static_cast<uint8_t>(in_buffer[1]) & 0x0F);
            out_msg.destination =
                (static_cast<uint8_t>(in_buffer[1]) >> 4) & 0x0F;
            out_msg.id = static_cast<uint8_t>(in_buffer[2]);
            out_msg.payload_size = payload_size;
            for (size_t i = 0; i < out_msg.payload_size; ++i) {
                out_msg.payload[i] =
                    static_cast<std::byte>(in_buffer[UART_HEADER_SIZE + i]);
            }

            return true;
        }
    };
}  // namespace simple_comm

#endif
