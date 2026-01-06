#ifndef __SIMPLE_CAN2_COMM_HPP
#define __SIMPLE_CAN2_COMM_HPP

#include <algorithm>
#include <array>
#include <span>
#include "Algorithms/fifo.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"

namespace simple_comm {
    inline namespace v1 {
        /*
        CAN2 Simple Comm Protocol Specification
        All Simple Comm metadata for CAN frames is stored in the 29-bit
        Extended ID (EID). Metadata is used to identify the Simple Comm
        protocol, route messages and allow deserialization of topic payloads.

        Extended ID (29 bits: EID[28:0]) layout (most-significant bits on left):
        bits: 28..26 25..22 21..18 17.....10 9.......0
              ┌───┬──────┬──────┬────────┬──────────┐
              │MTB│ DEST │ SRC  │topic_id│ unused   │
              │(3)│ (4)  │ (4)  │ (8)    │ (10)     │
              └───┴──────┴──────┴────────┴──────────┘

        fields:
        EID[28..26] : MAGIC_TRIBIT (3 bits)
        EID[25..22] : message destination (4 bits)
        EID[21..18] : message source (4 bits)
        EID[17..10] : topic_id (8 bits)
        EID[9..0]   : currently unused (10 bits)

        Note: Implementations must verify the MAGIC_TRIBIT in EID[28..26]
        before treating a frame as a Simple Comm message. The Standard ID
        (SID) is unused for Simple Comm frames when the EID is populated.

        
        UART Simple Comm Protocol Specification

        The UART protocol contains the same information as the CAN2 version of 
        simple comm, but with a slightly different format. Each message consists of
        4-byte header + payload + 2-byte trailer.

        Header (4 bytes): MAGIC_TRIBIT (8b), DEST/SRC (1 byte), topic_id (8b), length (8b)
        Payload: 0..8 bytes
        Trailer: 2-byte checksum (LSB first)

        Bytes (UART):
        index:  0     1            2         3        4 .. (4+N-1)   4+N   5+N
                ┌───┬────────────┬──────────┬────────┬────────────┬─────┬─────┐
        byte:   │MTB│ DEST | SRC │ topic_id │ length │   payload  │ cksL│ cksH│
        bits:   │(8)│ 4b   | 4b  │  (8b)    │  (8b)  │(0..8 bytes)│(8b) │(8b) │
                └───┴────────────┴──────────┴────────┴────────────┴─────┴─────┘
        */

        constexpr std::byte MAGIC_TRIBIT = std::byte {0x1};
        constexpr size_t MAX_PAYLOAD_SIZE = 8;

        constexpr size_t UART_HEADER_SIZE = 4;
        constexpr size_t UART_TRAILER_SIZE = 2;
        constexpr size_t UART_MAX_MESSAGE_SIZE =
            UART_HEADER_SIZE + MAX_PAYLOAD_SIZE + UART_TRAILER_SIZE;

        /**
         * @brief SimpleMessage structure representing a single data message.
         *  @note Payload serialization/deserialization is assumed to be handled
         * elsewhere.
         */
        struct SimpleMessage {
            uint8_t source;
            uint8_t destination;
            uint8_t topic_id;
            uint8_t payload_size;
            std::array<std::byte, MAX_PAYLOAD_SIZE> payload;
        };

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

           public:
            explicit SimpleComm()
                : message_rx_buffer(),
                  uart_temp_rx_buffer {},
                  uart_rx_fsm_state(UARTFSMState::WAIT_FOR_TRIBIT) {};

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
                    // All Simple Comm metadata is stored in the 29-bit EID.
                    // Layout (EID[28:0]): [MAGIC(3)][DEST(4)][SRC(4)][TOPIC(8)][UNUSED(10)]
                    uint8_t magic =
                        static_cast<uint8_t>((frame.eid >> 26) & 0x07);
                    if (magic != static_cast<uint8_t>(MAGIC_TRIBIT)) {
                        // Not a Simple Comm frame
                        return;
                    }

                    SimpleMessage new_msg;
                    new_msg.destination =
                        static_cast<uint8_t>((frame.eid >> 22) & 0x0F);
                    new_msg.source =
                        static_cast<uint8_t>((frame.eid >> 18) & 0x0F);
                    new_msg.topic_id =
                        static_cast<uint8_t>((frame.eid >> 10) & 0xFF);
                    new_msg.payload_size = frame.dlc;
                    std::copy(frame.payload.begin(),
                              frame.payload.begin() + frame.dlc,
                              new_msg.payload.begin());
                    message_rx_buffer.push(new_msg);
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
             *  - Header byte indexes: [0]=MTB (MAGIC_TRIBIT), [1]=DEST|SRC, [2]=topic_id, [3]=length.
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
                        if (uart_temp_rx_buffer[0] ==
                            static_cast<uint8_t>(MAGIC_TRIBIT)) {
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
                            new_msg.topic_id =
                                static_cast<uint8_t>(uart_temp_rx_buffer[2]);
                            new_msg.payload_size =
                                static_cast<uint8_t>(uart_temp_rx_buffer[3]);

                            for (size_t i = 0; i < new_msg.payload_size; ++i) {
                                new_msg.payload[i] = static_cast<std::byte>(
                                    uart_temp_rx_buffer[UART_HEADER_SIZE + i]);
                            }
                            message_rx_buffer.push(new_msg);

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

            /**
             * @brief Formats a SimpleMessage for UART transmission.
             * 
             * This function simply formats the contents of SimpleMessage and 
             * does not modify its contents at all. Serialization of payload
             * data is assumed to be handled elsewhere. Errors in SimpleMessage
             * metadata will result in an assertion error. See preconditions.
             * 
             * @param msg The SimpleMessage to format.
             * @pre msg.payload_size <= MAX_PAYLOAD_SIZE
             * @pre msg.destination <= 0x0F
             * @pre msg.source <= 0x0F
             * 
             * @param out_buffer The output buffer to store the formatted UART message.
             * @pre out_buffer.size() >= UART_MAX_MESSAGE_SIZE
             * @param out_length The length of the formatted message.
             */
            void format_uart_message(const SimpleMessage& msg,
                                     std::span<std::byte>& out_buffer,
                                     size_t& out_length) {
                ASSERT(msg.payload_size <= MAX_PAYLOAD_SIZE,
                       "Payload size exceeds maximum allowed.");
                ASSERT(msg.destination <= 0x0F,
                       "Destination ID exceeds 4-bit limit.");
                ASSERT(msg.source <= 0x0F, "Source ID exceeds 4-bit limit.");
                ASSERT(out_buffer.size() >= UART_MAX_MESSAGE_SIZE,
                       "Output buffer too small for UART message.");

                out_buffer[0] = MAGIC_TRIBIT;
                out_buffer[1] = std::byte {static_cast<uint8_t>(
                    (msg.destination << 4) | (msg.source & 0x0F))};
                out_buffer[2] = std::byte {msg.topic_id};
                out_buffer[3] = std::byte {msg.payload_size};
                std::copy_n(msg.payload.begin(), msg.payload_size,
                            out_buffer.begin() + UART_HEADER_SIZE);

                uint16_t checksum = 0;
                for (size_t i = 0; i < UART_HEADER_SIZE + msg.payload_size;
                     ++i) {
                    checksum += static_cast<uint8_t>(out_buffer[i]);
                }
                // TODO: 16-bit Checksum
                out_buffer[UART_HEADER_SIZE + msg.payload_size] =
                    std::byte {static_cast<uint8_t>(checksum & 0xFF)};
                out_buffer[UART_HEADER_SIZE + msg.payload_size + 1] =
                    std::byte {static_cast<uint8_t>((checksum >> 8) & 0xFF)};

                out_length =
                    UART_HEADER_SIZE + msg.payload_size + UART_TRAILER_SIZE;
            }

            /**
             * @brief Formats a SimpleMessage for CAN transmission.
             * 
             * This function simply formats the contents of SimpleMessage and
             * does not modify its contents at all. Serialization of payload
             * data is assumed to be handled elsewhere. Errors in SimpleMessage
             * metadata will result in an assertion error. See preconditions.
             * 
             * @param msg The SimpleMessage to format.
             * @pre msg.payload_size <= MAX_PAYLOAD_SIZE
             * @pre msg.destination <= 0x0F
             * @pre msg.source <= 0x0F
             * @param frame The CAN frame to store the formatted CAN message.
             */
            void format_can_message(const SimpleMessage& msg,
                                    MW_CAN::CANFrame& frame) {
                ASSERT(msg.payload_size <= MAX_PAYLOAD_SIZE,
                       "Payload size exceeds maximum allowed.");
                ASSERT(msg.destination <= 0x0F,
                       "Destination ID exceeds 4-bit limit.");
                ASSERT(msg.source <= 0x0F, "Source ID exceeds 4-bit limit.");

                // Pack all metadata into the 29-bit Extended ID (EID).
                // Layout (EID[28:0]): [MAGIC(3)][DEST(4)][SRC(4)][TOPIC(8)][UNUSED(10)]
                uint32_t eid = 0;
                eid |= (static_cast<uint32_t>(
                            static_cast<uint8_t>(MAGIC_TRIBIT) & 0x07)
                        << 26);
                eid |= (static_cast<uint32_t>(msg.destination & 0x0F) << 22);
                eid |= (static_cast<uint32_t>(msg.source & 0x0F) << 18);
                eid |= (static_cast<uint32_t>(msg.topic_id) << 10);

                frame.eid = eid;
                frame.sid = 0;  // clear/unused when using extended ID
                frame.is_extended_id = true;
                frame.dlc = msg.payload_size;
                std::copy_n(msg.payload.begin(), msg.payload_size,
                            frame.payload.begin());
            }
        };
    }  // namespace v1
}  // namespace simple_comm

#endif