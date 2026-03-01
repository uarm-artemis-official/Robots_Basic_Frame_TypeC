#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "../Algorithms/fifo.hpp"
#include "../ISRs/can_isr.hpp"
#include "../ISRs/uart_isr.hpp"
#include "../Middleware/middleware_interfaces.hpp"
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

        bool push(std::span<const std::byte> data, const MessageMeta& meta) {
            bool data_pushed = data_fifo.push(data.first(meta.payload_length));
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

        bool pop(std::span<std::byte> data, MessageMeta& meta) {
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
        /// Dependencies.
        uc_uart::UC_UART<MAX_SINGLE_MESSAGE_SIZE>& uc_uart_ref;
        MW_UART::IUART& uart_ref;

        /// State variables for UART receive complete routine function.
        bool is_valid_uart_magic = false;
        bool is_valid_uart_header = false;
        size_t uart_expected_payload_length = 0;
        uint8_t rx_buffer[MAX_SINGLE_MESSAGE_SIZE + uc_uart::HEADER_SIZE +
                          uc_uart::TRAILER_SIZE] {0};

        MessageFIFO<protocol::TopicMessageMeta, MAX_SINGLE_MESSAGE_SIZE * 4>
            receive_fifo, send_fifo;

       public:
        explicit UARTComm(
            uc_uart::UC_UART<MAX_SINGLE_MESSAGE_SIZE>& _uc_uart_ref,
            MW_UART::IUART& _uart_ref)
            : uc_uart_ref(_uc_uart_ref), uart_ref(_uart_ref) {};

        bool init() { return true; }

        /** 
         * @brief Start receiving data on the UART interface.
         * 
         * This method should be registered as a UART ISR init function.
         * 
         * @param uart Reference to the UART interface.
         * @return true if reception was started successfully, false otherwise.
         */
        bool start_receive(MW_UART::IUART& uart) {
            return uart.receive_data(MW_UART::Peripheral::UART1, rx_buffer, 1);
        }

        /**
         * @brief UART receive complete ISR handler.
         * 
         * This method handles the UART receive complete interrupt and 
         * should be registered as a UART ISR routine function.
         * It processes incoming data, checks for valid headers,
         * and stores complete messages into the receive FIFO.
         * 
         * @param uart Reference to the UART interface.
         * @param peripheral The UART peripheral that triggered the interrupt.
         */
        void on_receive_complete(MW_UART::IUART& uart,
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

        /**
         * @brief Queue a message for sending over UART.
         * 
         * TODO: Remove and replace with std::span overload only.
         * 
         * @param payload Pointer to the message payload data.
         * @param meta Metadata associated with the message.
         */
        void queue_send_message(const uint8_t* payload,
                                protocol::TopicMessageMeta meta) {
            send_fifo.push(payload, meta);
        }

        /**
         * @brief Overload of queue_send_message to accept std::span.
         * 
         * @param payload std::span containing the message payload data.
         * @param meta Metadata associated with the message.
         */
        void queue_send_message(std::span<const std::byte> payload,
                                protocol::TopicMessageMeta meta) {
            send_fifo.push(payload, meta);
        }

        /**
         * @brief Send the next message frame from the send FIFO over UART.
         * 
         * This method retrieves the next message from the send FIFO,
         * formats it using the UC_UART module, and sends it over the UART interface.
         * Since messages do not require segmentation, a single frame is sent each time this
         * method is called.
         * 
         * TODO: Make method non-blocking.
         */
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

        /**
         * @brief Retrieve a received message from the receive FIFO.
         * 
         * TODO: Remove and replace with std::span overload only.
         * 
         * @param payload Pointer to the buffer where the message payload will be copied.
         * @param meta Reference to the metadata object where the message metadata will be stored.
         * @return true if a message was successfully retrieved, false otherwise.
         */
        bool get_message(uint8_t* payload, protocol::TopicMessageMeta& meta) {
            return receive_fifo.pop(payload, meta);
        }

        /**
         * @brief Overload of get_message to accept std::span.
         * 
         * @param payload std::span containing the message payload data.
         * @param meta Reference to the metadata object where the message metadata will be stored.
         * @return true if a message was successfully retrieved, false otherwise.
         */
        bool get_message(std::span<std::byte> payload,
                         protocol::TopicMessageMeta& meta) {
            return receive_fifo.pop(payload, meta);
        }
    };

    // CANComm class has been removed as it depended on can2_tp which has been deprecated.
    // TODO: Implement a new CAN communication class if needed.

}  // namespace comm

#endif