#ifndef __SIMPLE_CAN2_COMM_HPP
#define __SIMPLE_CAN2_COMM_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <span>
#include <tuple>
#include <type_traits>
#include "Algorithms/fifo.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"

namespace simple_comm {
    // Single data message constants and types.
    constexpr std::byte DATA_MAGIC_TRIBIT = std::byte {0x1};

    // Command message constants and types.
    constexpr std::byte COMMAND_MAGIC_TRIBIT = std::byte {0x3};
    constexpr uint32_t ACK_TIMEOUT_MS = 500;
    constexpr size_t MAX_RETRIES = 3;

    enum class MessageType : std::underlying_type_t<std::byte> {
        DATA =
            static_cast<std::underlying_type_t<std::byte>>(DATA_MAGIC_TRIBIT),
        COMMAND = static_cast<std::underlying_type_t<std::byte>>(
            COMMAND_MAGIC_TRIBIT),
    };

    template <typename T>
    concept CommandMessage =
        std::is_trivially_copyable_v<T> &&
        requires(T msg, std::span<std::byte, T::SERIALIZED_SIZE> dst,
                 std::span<const std::byte, T::SERIALIZED_SIZE> src) {
        requires std::same_as<decltype(T::SERIALIZED_SIZE), const size_t>;
        requires std::same_as<decltype(T::MESSAGE_TYPE), const MessageType>;
        requires std::same_as<decltype(T::MESSAGE_ID), const uint8_t>;
        {T::serialize_payload(msg, dst)}->std::same_as<bool>;
        {T::deserialize_payload(src, msg)}->std::same_as<bool>;
    };

    // General protocol constants and types.
    constexpr size_t MAX_PAYLOAD_SIZE = 8;

    constexpr size_t UART_HEADER_SIZE = 4;
    constexpr size_t UART_TRAILER_SIZE = 2;
    constexpr size_t UART_MAX_MESSAGE_SIZE =
        UART_HEADER_SIZE + MAX_PAYLOAD_SIZE + UART_TRAILER_SIZE;

    enum class NodeID : uint8_t {
        Telemetry = 1,
        Chassis = 2,
        Gimbal = 3,
        MiniPC = 4,
        All = 0x0F
    };

    struct SimpleMessage {
        MessageType message_type;
        uint8_t source;
        uint8_t destination;
        uint8_t id;
        uint8_t payload_size;
        std::array<std::byte, MAX_PAYLOAD_SIZE> payload;
    };

    // Template codec that serializes/deserializes SimpleMessage to/from
    // a user-provided CAN POD type (CANPod). CANPod must be a POD with
    // members: uint32_t eid; uint16_t sid; bool is_extended_id; uint8_t dlc; std::array<std::byte,8> payload;
    class SimpleCommCodec {
       public:
        bool is_recognized_magic(uint8_t magic) const {
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
        void to_can_message(const SimpleMessage& msg, CANPod& out_frame) const {
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
        [[nodiscard]] bool from_can_message(const CANPod& frame,
                                            SimpleMessage& out_msg) const {
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
        uint16_t calc_uart_checksum(const std::byte* buf, size_t len) const {
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
        void to_uart_bytes(const SimpleMessage& msg,
                           std::span<std::byte> out_buffer,
                           size_t& out_length) const {
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
        bool from_uart_bytes(std::span<const std::byte> in_buffer,
                             SimpleMessage& out_msg) const {
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

    inline namespace v1 {
        /*
        CAN2 Simple Comm Protocol Specification
        All Simple Comm metadata for CAN frames is stored in the 29-bit
        Extended ID (EID). Metadata is used to identify the Simple Comm
        protocol, route messages and allow deserialization of topic payloads.

        Extended ID (29 bits: EID[28:0]) layout (most-significant bits on left):
        bits: 28..26 25..22 21..18 17.....10 9.......0
              ┌───┬──────┬──────┬────────┬──────────┐
              │MTB│ DEST │ SRC  │  id    │ unused   │
              │(3)│ (4)  │ (4)  │  (8)   │ (10)     │
              └───┴──────┴──────┴────────┴──────────┘

        fields:
        EID[28..26] : MAGIC_TRIBIT (3 bits)
        EID[25..22] : message destination (4 bits)
         - Each node in the communication network is assigned a unique 4-bit ID 
           which is used in the DEST field to specify the intended recipient of 
           a message.
         - If a message needs to be sent to multiple recipients, the message can 
           be sent multiple times with the same payload and source but different 
           DEST values. There is no broadcast/multicast mechanism at the protocol 
           level, but it can be implemented at a higher level if needed (e.g. by 
           reserving a special DEST value to indicate broadcast and having all 
           nodes process messages with that DEST).
        EID[21..18] : message source (4 bits)
         - See notes on DEST field. Source field is used to identify the sender 
           of a message.
        EID[17..10] : id (8 bits)
        EID[9..0]   : currently unused (10 bits)

        Note: Implementations must verify the MAGIC_TRIBIT in EID[28..26]
        before treating a frame as a Simple Comm message. The Standard ID
        (SID) is unused for Simple Comm frames when the EID is populated.
        There are two main categories of messages: single data messages 
        (for sensor readings, etc.) and command messages (for control commands, etc.)
        and each have their own MAGIC_TRIBIT value, however, they both have the
        same protocol format.
        
        UART Simple Comm Protocol Specification

        The UART protocol contains the same information as the CAN2 version of 
        simple comm, but with a slightly different format. Each message consists of
        4-byte header + payload + 2-byte trailer.

        Header (4 bytes): MAGIC_TRIBIT (8b), DEST/SRC (1 byte), id (8b), length (8b)
        Payload: 0..8 bytes
        Trailer: 2-byte checksum (LSB first)

        Bytes (UART):
        index:  0     1            2         3        4 .. (4+N-1)   4+N   5+N
                ┌───┬────────────┬──────────┬────────┬────────────┬─────┬─────┐
        byte:   │MTB│ DEST | SRC │    id    │ length │   payload  │ cksL│ cksH│
        bits:   │(8)│ 4b   | 4b  │   (8b)   │  (8b)  │(0..8 bytes)│(8b) │(8b) │
                └───┴────────────┴──────────┴────────┴────────────┴─────┴─────┘
        */

        // Implementation specific types.
        /**
         * @brief UART receive finite state machine states.
         */
        enum class UARTFSMState {
            WAIT_FOR_TRIBIT,
            WAIT_FOR_HEADER,
            WAIT_FOR_REST_MESSAGE
        };

        /**
         * @brief Simple communication module for sending single data messages.
         * 
         * This module supports sending and receiving messages over CAN and UART.
         * Each message consists of a source ID, destination ID, topic ID, and a payload.
         * Messages received are stored in a FIFO buffer for later processing.
         * 
         * @tparam RxFIFOSize Size of the receive FIFO buffer.
         */
        template <size_t RxFIFOSize>
        class SimpleComm {
           private:
            dsa::RingBuffer<SimpleMessage, RxFIFOSize> message_rx_buffer;
            std::array<uint8_t, UART_MAX_MESSAGE_SIZE> uart_temp_rx_buffer;
            UARTFSMState uart_rx_fsm_state;
            SimpleCommCodec codec;

           public:
            explicit SimpleComm()
                : message_rx_buffer(),
                  uart_temp_rx_buffer {},
                  uart_rx_fsm_state(UARTFSMState::WAIT_FOR_TRIBIT),
                  codec() {};

            /**
             * @brief CAN message pending interrupt service routine.
             * 
             * This function should be called from the CAN message pending ISR.
             * It extracts the SimpleMessage from the CAN frame and pushes it
             * to the RX buffer.
             * 
             * @param bus The CAN bus where the message was received.
             * @param frame The received CAN frame.
             */
            void can_isr_message_pending(MW_CAN::BUS bus,
                                         MW_CAN::CANFrame frame) {
                if (bus == MW_CAN::BUS::CAN_2B) {
                    SimpleMessage new_msg;
                    bool success = codec.from_can_message<MW_CAN::CANFrame>(
                        frame, new_msg);
                    if (success) {
                        // TODO: (Possible) Add error handling for RX buffer overflow (e.g. return a status, set a flag, etc.)
                        (void) message_rx_buffer.push(new_msg);
                    }
                }
            }

            /**
             * @brief Initializes the UART receive interrupt service routine.
             * 
             * This function sets up the UART to begin receiving data and
             * initializes the FSM state for parsing incoming messages. Simple
             * Comm UART operates on UART1 peripheral (which is the four-pin 
             * UART on the Type-C).
             * 
             * @param uart Reference to the UART interface.
             * @return true if initialization was successful, false otherwise.
             */
            bool uart_isr_init(MW_UART::IUART& uart) {
                bool res = uart.receive_data(MW_UART::Peripheral::UART1,
                                             uart_temp_rx_buffer.data(), 1);
                uart_rx_fsm_state = UARTFSMState::WAIT_FOR_TRIBIT;
                return res;
            }

            /**
             * @brief UART receive complete interrupt service routine.
             * 
             * This function should be called from the UART receive complete ISR.
             * It implements a finite state machine (FSM) to parse incoming UART
             * messages according to the simple comm protocol. The transition function
             * looks like the following:
             * 
             * UART ISR FSM transition table for `UARTFSMState` used in `uart_isr_receive_complete`
             * +----------------------+----------------------------------------------+-----------------------------------------------------------+-------------------------+
             * | Current State        | Event / Condition                            | Action(s)                                                 | Next State              |
             * +----------------------+----------------------------------------------+-----------------------------------------------------------+-------------------------+
             * | WAIT_FOR_TRIBIT      | received byte == MAGIC_TRIBIT                | receive (UART_HEADER_SIZE - 1) bytes into buffer[1..3]    | WAIT_FOR_HEADER         |
             * |                      | received byte != MAGIC_TRIBIT                | receive 1 byte into buffer[0] (keep waiting)              | WAIT_FOR_TRIBIT         |
             * +----------------------+----------------------------------------------+-----------------------------------------------------------+-------------------------+
             * | WAIT_FOR_HEADER      | header received AND 0 < payload_size <= MAX  | receive (payload_size + UART_TRAILER_SIZE) into buffer    | WAIT_FOR_REST_MESSAGE   |
             * |                      | header received AND (payload_size == 0 OR >MAX) | receive 1 byte into buffer[0] (restart sync)          | WAIT_FOR_TRIBIT         |
             * +----------------------+----------------------------------------------+-----------------------------------------------------------+-------------------------+
             * | WAIT_FOR_REST_MESSAGE| payload+trailer received                      | calc checksum over header+payload; compare with trailer   | WAIT_FOR_TRIBIT         |
             * |                      | checksum == received_checksum                | parse src/dest/topic/payload; push message to RX buffer;  | WAIT_FOR_TRIBIT         |
             * |                      |                                              | call receive_data(...,1)                                   |                         |
             * |                      | checksum != received_checksum                | discard; call receive_data(...,1)                         | WAIT_FOR_TRIBIT         |
             * +----------------------+----------------------------------------------+-----------------------------------------------------------+-------------------------+
             * 
             * Notes:
             *  - Header byte indexes: [0]=MTB (MAGIC_TRIBIT), [1]=DEST|SRC, [2]=id, [3]=length.
             *  - Trailer is 2-byte checksum (LSB first) located at UART_HEADER_SIZE + payload_size and +1.
             *  - All flows end by re-arming reception for 1 byte and returning to WAIT_FOR_TRIBIT.
             * 
             * @param uart Reference to the UART interface.
             * @param peripheral The UART peripheral that triggered the ISR.
             */
            void uart_isr_receive_complete(MW_UART::IUART& uart,
                                           MW_UART::Peripheral peripheral) {
                if (peripheral != MW_UART::Peripheral::UART1) {
                    return;
                }

                switch (uart_rx_fsm_state) {
                    case UARTFSMState::WAIT_FOR_TRIBIT: {
                        if (codec.is_recognized_magic(uart_temp_rx_buffer[0])) {
                            // Move to receive header state
                            uart.receive_data(MW_UART::Peripheral::UART1,
                                              uart_temp_rx_buffer.data() + 1,
                                              UART_HEADER_SIZE - 1);
                            uart_rx_fsm_state = UARTFSMState::WAIT_FOR_HEADER;
                        } else {
                            // Continue waiting for tribit
                            uart.receive_data(MW_UART::Peripheral::UART1,
                                              uart_temp_rx_buffer.data(), 1);
                        }
                        break;
                    }
                    case UARTFSMState::WAIT_FOR_HEADER: {
                        uint8_t payload_size =
                            static_cast<uint8_t>(uart_temp_rx_buffer[3]);
                        if (0 < payload_size &&
                            payload_size <= MAX_PAYLOAD_SIZE) {
                            // Move to receive payload state
                            uart.receive_data(
                                MW_UART::Peripheral::UART1,
                                uart_temp_rx_buffer.data() + UART_HEADER_SIZE,
                                payload_size + UART_TRAILER_SIZE);
                            uart_rx_fsm_state =
                                UARTFSMState::WAIT_FOR_REST_MESSAGE;
                        } else {
                            // Invalid payload size, restart reception
                            uart.receive_data(MW_UART::Peripheral::UART1,
                                              uart_temp_rx_buffer.data(), 1);
                            uart_rx_fsm_state = UARTFSMState::WAIT_FOR_TRIBIT;
                        }
                        break;
                    }
                    case UARTFSMState::WAIT_FOR_REST_MESSAGE: {
                        uint8_t payload_size =
                            static_cast<uint8_t>(uart_temp_rx_buffer[3]);
                        uint16_t calc_checksum = 0;
                        for (size_t i = 0; i < UART_HEADER_SIZE + payload_size;
                             ++i) {
                            calc_checksum +=
                                static_cast<uint8_t>(uart_temp_rx_buffer[i]);
                        }

                        uint16_t received_checksum =
                            static_cast<uint16_t>(
                                uart_temp_rx_buffer[UART_HEADER_SIZE +
                                                    payload_size]) |
                            (static_cast<uint16_t>(
                                 uart_temp_rx_buffer[UART_HEADER_SIZE +
                                                     payload_size + 1])
                             << 8);

                        if (calc_checksum == received_checksum) {
                            // Valid message received, process it below
                            SimpleMessage new_msg;
                            new_msg.source =
                                (static_cast<uint8_t>(uart_temp_rx_buffer[1]) &
                                 0x0F);
                            new_msg.destination =
                                (static_cast<uint8_t>(uart_temp_rx_buffer[1]) >>
                                 4) &
                                0x0F;
                            new_msg.id =
                                static_cast<uint8_t>(uart_temp_rx_buffer[2]);
                            new_msg.payload_size =
                                static_cast<uint8_t>(uart_temp_rx_buffer[3]);

                            for (size_t i = 0; i < new_msg.payload_size; ++i) {
                                new_msg.payload[i] = static_cast<std::byte>(
                                    uart_temp_rx_buffer[UART_HEADER_SIZE + i]);
                            }
                            // TODO: (Possible) Add error handling for RX buffer overflow (e.g. return a status, set a flag, etc.)
                            (void) message_rx_buffer.push(new_msg);

                            uart_rx_fsm_state = UARTFSMState::WAIT_FOR_TRIBIT;
                        }

                        uart.receive_data(MW_UART::Peripheral::UART1,
                                          uart_temp_rx_buffer.data(), 1);
                        uart_rx_fsm_state = UARTFSMState::WAIT_FOR_TRIBIT;
                        break;
                    }
                    default:
                        ASSERT(false, "Invalid UART FSM state.");
                        break;
                }
            }

            /**
             * @brief Retrieves the next received SimpleMessage from the RX buffer.
             * 
             * The RX buffer operates as RingBuffer FIFO, if there is an overflow,
             * the oldest messages will be discarded. If there are no messages
             * available, this function will return false.
             * 
             * @param dst Reference to store the retrieved SimpleMessage.
             */
            bool get_rx_message(SimpleMessage& dst) {
                return message_rx_buffer.pop(dst);
            }
        };
    }  // namespace v1
}  // namespace simple_comm

#endif