#ifndef __SIMPLE_CAN2_COMM_HPP
#define __SIMPLE_CAN2_COMM_HPP

#include <algorithm>
#include <array>
#include <span>
#include "Algorithms/fifo.hpp"
#include "fifo.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"

namespace simple_comm {
    inline namespace v1 {
        constexpr std::byte MAGIC_TRIBIT = std::byte {0x1};
        constexpr size_t MAX_PAYLOAD_SIZE = 8;

        constexpr size_t UART_HEADER_SIZE = 4;
        constexpr size_t UART_TRAILER_SIZE = 2;
        constexpr size_t UART_MAX_MESSAGE_SIZE =
            UART_HEADER_SIZE + MAX_PAYLOAD_SIZE + UART_TRAILER_SIZE;

        struct SimpleMessage {
            uint8_t source;
            uint8_t destination;
            uint8_t topic_id;
            uint8_t payload_size;
            std::array<std::byte, MAX_PAYLOAD_SIZE> payload;
        };

        enum class UARTFSMState {
            WAIT_FOR_TRIBIT,
            WAIT_FOR_HEADER,
            WAIT_FOR_REST_MESSAGE
        };

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

            void can_isr_message_pending(MW_CAN::BUS bus,
                                         MW_CAN::CANFrame frame) {
                if (bus == MW_CAN::BUS::CAN_2B) {
                    SimpleMessage new_msg;
                    new_msg.destination =
                        static_cast<uint8_t>((frame.sid >> 4) & 0x0F);
                    new_msg.source =
                        static_cast<uint8_t>((frame.sid >> 0) & 0x0F);
                    new_msg.topic_id =
                        static_cast<uint8_t>((frame.eid >> 11) & 0xFF);
                    new_msg.payload_size = frame.dlc;
                    std::copy(frame.payload.begin(),
                              frame.payload.begin() + frame.dlc,
                              new_msg.payload.begin());
                    message_rx_buffer.push(new_msg);
                }
            }

            bool uart_isr_init(MW_UART::IUART& uart) {
                bool res = uart.receive_data(MW_UART::Peripheral::UART1,
                                             uart_temp_rx_buffer.data(), 1);
                uart_rx_fsm_state = UARTFSMState::WAIT_FOR_TRIBIT;
                return res;
            }

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

            bool get_rx_message(SimpleMessage& dst) {
                return message_rx_buffer.pop(dst);
            }

            void format_uart_message(
                const SimpleMessage& msg,
                std::span<std::byte, UART_MAX_MESSAGE_SIZE>& out_buffer,
                size_t& out_length) {
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

            void format_can_message(const SimpleMessage& msg,
                                    MW_CAN::CANFrame& frame) {
                frame.sid =
                    (static_cast<uint32_t>(MAGIC_TRIBIT) << 8) |
                    ((static_cast<uint32_t>(msg.destination) & 0x0F) << 4) |
                    (static_cast<uint32_t>(msg.source) & 0x0F);
                frame.eid = (static_cast<uint32_t>(msg.topic_id) << 11);
                frame.dlc = msg.payload_size;
                std::copy_n(msg.payload.begin(), msg.payload_size,
                            frame.payload.begin());
            }
        };
    }  // namespace v1
}  // namespace simple_comm

#endif