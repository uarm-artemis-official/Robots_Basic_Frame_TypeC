#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "../Algorithms/fifo.hpp"
#include "../ISRs/can_isr.hpp"
#include "../ISRs/uart_isr.hpp"
#include "../Middleware/middleware_interfaces.hpp"
#include "../Modules/can2_tp.hpp"
#include "../Modules/uc_uart.hpp"
#include "Modules/comm_protocol_interface.hpp"

// TODO: Integrate communication system into robot.

namespace comm {
    /**
     * @brief FIFO for storing messages with associated metadata.
     * 
     * This is a composite FIFO that uses VarFIFO for storing raw message data
     * and a RingBuffer for storing associated metadata for each message. It is
     * used to maintain information while messages are being processed or queued
     * for transmission. It utilizes VarFIFO and RingBuffer data structures from
     * the dsa namespace. Therefore, it has limited capacity which should be set
     * accordingly to allow for more than expected message loads.
     * 
     * This FIFO has two limitations in regards to how many messages it can store.
     * First, is the total size of all messages stored in the VarFIFO. This means 
     * the sum of all payloads (in bytes) that need to be stored must be lower 
     * than VAR_FIFO_SIZE. Second, is the total number of individual messages that 
     * can be stored in the RingBuffer must be lower than MESSAGE_FIFO_SIZE.
     * 
     * @tparam MessageMeta Type of metadata associated with each message.
     * @tparam VAR_FIFO_SIZE Size of the underlying VarFIFO for message data (in bytes).
     * @tparam MESSAGE_FIFO_SIZE Number of messages that can be stored in the FIFO at one time.
     */
    template <typename MessageMeta, size_t VAR_FIFO_SIZE = 1024,
              size_t MESSAGE_FIFO_SIZE = 20>
    class MessageFIFO {
       private:
        dsa::VarFIFO<VAR_FIFO_SIZE, MESSAGE_FIFO_SIZE> data_fifo;
        dsa::RingBuffer<MessageMeta, MESSAGE_FIFO_SIZE> meta_buffer;

       public:
        /**
         * @brief Push a message and its metadata into the FIFO.
         * 
         * The message data being pushed into the FIFO must be less than or equal to
         * VAR_FIFO_SIZE. Otherwise, an assertion error will occur. If there is not enough
         * space in either the data FIFO or metadata buffer, the push will fail and
         * return false, but there will be no assertion error.
         * 
         * @param data Pointer to the message data.
         * @param data_length Length of the message data.
         * @param meta Metadata associated with the message.
         * @return true if the message and metadata were successfully pushed, false otherwise.
         */
        bool push(const uint8_t* data, size_t data_length,
                  const MessageMeta& meta) {
            bool data_pushed = data_fifo.push(data, data_length);
            bool meta_pushed = meta_buffer.push(meta);
            return data_pushed && meta_pushed;
        }

        /**
         * @brief Pop a message and its metadata from the FIFO.
         * 
         * There are no checks for buffer overflows so the caller must ensure
         * the destination data buffer is large enough to hold the message.
         * The only guarantee is that a single message will not exceed VAR_FIFO_SIZE.
         * Other guarantees must be enforced by the caller.
         * 
         * @param data Pointer to the buffer where the message data will be copied to.
         * @param meta Reference to the metadata object where the metadata will be stored.
         * @return true if the message and metadata were successfully popped, false otherwise.
         */
        bool pop(uint8_t* data, MessageMeta& meta) {
            bool meta_popped = meta_buffer.pop(meta);
            size_t data_length = 0;
            bool data_popped = data_fifo.pop(data, data_length);
            return meta_popped && data_popped;
        }
    };

    /**
     * @brief Communication subsystem handling UART communication.
     * 
     * Messages are sent using UC_UART module and automatically received
     * and parsed from UART interrupts. Two FIFOs are maintained for sending
     * and receiving messages. Receive FIFO is populated from UART ISR with assembled
     * messages. These messages can be consumed by using get_message(). Send FIFO
     * is populated using queue_send_message() and messages are sent out using
     * send_next_frame() presumably at the next application tick.
     * 
     * The processing in the ISR routine may be quite large and may need to be optimized
     * or offloaded to a task in the future if performance becomes an issue.
     * 
     * TODO: Profile ISR routine performance to get an estimate of execution time.
     * 
     * @note There currently is no support for flow control frames. Futher development is
     * required for that.
     * 
     * @tparam MAX_SINGLE_MESSAGE_SIZE Maximum size of a single message payload.
     */
    template <size_t MAX_SINGLE_MESSAGE_SIZE = 256>
    class UARTComm {
        static_assert(MAX_SINGLE_MESSAGE_SIZE > 0,
                      "MAX_SINGLE_MESSAGE_SIZE must be greater than 0");

       private:
        uc_uart::UC_UART<MAX_SINGLE_MESSAGE_SIZE>& uc_uart_ref;
        MW_UART::IUART& uart_ref;
        isr::uart::UART_ISR& uart_isr_ref;
        MessageFIFO<protocol::TopicMessageMeta, MAX_SINGLE_MESSAGE_SIZE * 4>
            receive_fifo, send_fifo;

        bool is_valid_uart_magic = false;
        bool is_valid_uart_header = false;
        size_t uart_expected_payload_length = 0;
        uint8_t rx_buffer[MAX_SINGLE_MESSAGE_SIZE + uc_uart::HEADER_SIZE +
                          uc_uart::TRAILER_SIZE] {0};

       public:
        explicit UARTComm(
            uc_uart::UC_UART<MAX_SINGLE_MESSAGE_SIZE>& _uc_uart_ref,
            MW_UART::IUART& _uart_ref, isr::uart::UART_ISR& _uart_isr_ref)
            : uc_uart_ref(_uc_uart_ref),
              uart_ref(_uart_ref),
              uart_isr_ref(_uart_isr_ref) {};

        bool init() {
            bool register_init_success = uart_isr_ref.register_init(
                [this](MW_UART::IUART& uart_instance) {
                    return this->uart_isr_init(uart_instance);
                });
            bool register_rountine_success = uart_isr_ref.register_routine(
                isr::uart::ECallbacks::RECEIVE_COMPLETE,
                [this](MW_UART::IUART& uart_instance,
                       MW_UART::Peripheral peripheral) {
                    this->uart_isr_receive_complete(uart_instance, peripheral);
                });
            return register_init_success && register_rountine_success;
        }

        bool uart_isr_init(MW_UART::IUART& uart) {
            uart.receive_data(MW_UART::Peripheral::UART1, rx_buffer, 1);
            return true;
        }

        void uart_isr_receive_complete(MW_UART::IUART& uart,
                                       MW_UART::Peripheral peripheral) {
            if (peripheral == MW_UART::Peripheral::UART1) {
                if (is_valid_uart_magic) {
                    if (is_valid_uart_header) {
                        protocol::TopicMessageMeta meta =
                            uc_uart_ref.parse_meta_from_headers(rx_buffer);
                        receive_fifo.push(rx_buffer,
                                          uart_expected_payload_length, meta);
                        is_valid_uart_magic = false;
                        is_valid_uart_header = false;
                        uart_expected_payload_length = 0;
                        std::memset(rx_buffer, 0, sizeof(rx_buffer));
                    } else {
                        size_t payload_length = 0;
                        uc_uart::parse_length(rx_buffer, payload_length);
                        if (payload_length > MAX_SINGLE_MESSAGE_SIZE) {
                            // Invalid length, reset state
                            is_valid_uart_magic = false;
                            is_valid_uart_header = false;
                            uart_expected_payload_length = 0;
                            std::memset(rx_buffer, 0, sizeof(rx_buffer));
                            uart.receive_data(MW_UART::Peripheral::UART1,
                                              &rx_buffer[0], 1);
                            return;
                        } else {
                            uart_expected_payload_length = payload_length;
                            uart.receive_data(
                                MW_UART::Peripheral::UART1,
                                &rx_buffer[uc_uart::HEADER_SIZE],
                                payload_length + uc_uart::TRAILER_SIZE);
                            is_valid_uart_header = true;
                        }
                    }
                } else {
                    if (rx_buffer[0] == 0x71) {
                        uart.receive_data(MW_UART::Peripheral::UART1,
                                          &rx_buffer[1],
                                          uc_uart::HEADER_SIZE - 1);
                        is_valid_uart_header = true;
                    } else {
                        uart.receive_data(MW_UART::Peripheral::UART1,
                                          &rx_buffer[0], 1);
                    }
                }
            }
        }

        void queue_send_message(const uint8_t* payload,
                                protocol::TopicMessageMeta meta) {
            send_fifo.push(payload, meta);
        }

        void send_next_frame() {
            uint8_t payload[MAX_SINGLE_MESSAGE_SIZE];
            protocol::TopicMessageMeta meta;
            send_fifo.pop(payload, meta);
            uint8_t dst_buffer[MAX_SINGLE_MESSAGE_SIZE + uc_uart::HEADER_SIZE +
                               uc_uart::TRAILER_SIZE];
            (void) uc_uart_ref.format_send_data(payload, meta, dst_buffer);
            uart_ref.send_data(MW_UART::Peripheral::UART1, dst_buffer,
                               uc_uart::HEADER_SIZE + meta.payload_length +
                                   uc_uart::TRAILER_SIZE,
                               5);
        }

        bool get_message(uint8_t* payload, protocol::TopicMessageMeta& meta) {
            return receive_fifo.pop(payload, meta);
        }
    };

    template <size_t MAX_SINGLE_MESSAGE_SIZE = 256>
    class CANComm {
        static_assert(MAX_SINGLE_MESSAGE_SIZE > 0,
                      "MAX_SINGLE_MESSAGE_SIZE must be greater than 0");

       private:
        can2_tp::v1::CAN2TP<MAX_SINGLE_MESSAGE_SIZE>& can2tp_ref;
        MW_CAN::ICAN& can_ref;
        isr::can::CAN_ISR& can_isr_ref;
        MessageFIFO<protocol::TopicMessageMeta, MAX_SINGLE_MESSAGE_SIZE * 4>
            receive_fifo, send_fifo;

       public:
        explicit CANComm(
            can2_tp::v1::CAN2TP<MAX_SINGLE_MESSAGE_SIZE>& _can2tp_ref,
            isr::can::CAN_ISR& _can_isr_ref, MW_CAN::ICAN& _can_ref)
            : can2tp_ref(_can2tp_ref),
              can_isr_ref(_can_isr_ref),
              can_ref(_can_ref) {};

        bool init() {
            bool register_init_success =
                can_isr_ref.register_init([this](MW_CAN::ICAN& can_instance) {
                    return this->can_isr_init(can_instance);
                });
            bool register_rountine_success = can_isr_ref.register_routine(
                isr::can::ECallbacks::MESSAGE_PENDING,
                [this](MW_CAN::BUS bus, isr::can::CANFrame frame) {
                    this->can_isr_message_pending(bus, frame);
                });
            return register_init_success && register_rountine_success;
        }

        bool can_isr_init(MW_CAN::ICAN&) { return true; }

        void can_isr_message_pending(MW_CAN::BUS bus,
                                     isr::can::CANFrame frame) {
            if (bus == MW_CAN::BUS::CAN_2B) {
                protocol::FrameType frame_type =
                    can2_tp::v1::get_frame_type(frame.stdid);

                can2_tp::v1::CAN2BFrame can2b_frame {
                    frame.stdid, frame.extid, {}, frame.payload_length};
                std::memcpy(can2b_frame.payload, frame.payload,
                            frame.payload_length);

                switch (frame_type) {
                    case protocol::FrameType::SingleFrame: {
                        uint8_t new_frame_buffer[8];
                        bool is_new_frame = can2tp_ref.process_single_frame(
                            can2b_frame, new_frame_buffer);
                        if (is_new_frame) {
                            (void) receive_fifo.push(
                                new_frame_buffer, can2b_frame.length,
                                can2tp_ref.parse_meta_from_headers(
                                    can2b_frame.stdid, can2b_frame.extid));
                        }
                        break;
                    }
                    case protocol::FrameType::FirstFrame:
                        [[fallthrough]];
                    case protocol::FrameType::ConsecutiveFrame: {
                        (void) can2tp_ref.process_segment_frame(can2b_frame);
                        uint8_t
                            reassembled_message_buffer[MAX_SINGLE_MESSAGE_SIZE];
                        protocol::TopicMessageMeta message_meta;
                        bool has_new_message =
                            can2tp_ref.get_reassembled_message(
                                reassembled_message_buffer, message_meta);
                        if (has_new_message) {
                            (void) receive_fifo.push(
                                reassembled_message_buffer,
                                message_meta.payload_length, message_meta);
                        }
                        break;
                    }
                    default:
                        // TODO: Implement control flow frame response.
                        (void) 0;
                }
            }
        }

        void queue_send_message(const uint8_t* payload,
                                protocol::TopicMessageMeta meta) {
            send_fifo.push(payload, meta);
        }

        void send_next_frame() {
            can2_tp::CAN2BFrame next_frame;
            if (!can2tp_ref.is_sending_message()) {
                uint8_t next_payload[MAX_SINGLE_MESSAGE_SIZE];
                protocol::TopicMessageMeta next_meta;
                if (send_fifo.pop(next_payload, next_meta)) {
                    if (next_meta.payload_length <
                        can2_tp::MIN_SEGMENT_MESSAGE_LENGTH) {
                        (void) can2tp_ref.get_single_frame(
                            next_payload, next_meta.payload_length,
                            next_meta.destination, next_frame);
                    } else {
                        can2tp_ref.set_send_message(
                            next_payload, next_meta.payload_length,
                            next_meta.message_id, next_meta.destination);
                        can2tp_ref.get_next_send_fragment(next_frame);
                    }
                }
            }

            if (next_frame.stdid != 0 && next_frame.extid != 0 &&
                next_frame.length != 0) {
                can_ref.send_data(MW_CAN::BUS::CAN_2B, next_frame.stdid,
                                  next_frame.extid, next_frame.payload,
                                  next_frame.length);
            }
        }

        bool get_message(uint8_t* payload, protocol::TopicMessageMeta& meta) {
            return receive_fifo.pop(payload, meta);
        }
    };
}  // namespace comm

#endif