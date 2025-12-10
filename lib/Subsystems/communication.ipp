#ifndef __COMMUNICATION_IPP
#define __COMMUNICATION_IPP

#include <type_traits>
#include "Modules/can2_tp.hpp"
#include "Modules/message_center.ipp"
#include "communication.hpp"

// TODO: Implement support for control flow frames.
namespace comm {
    template <typename TMessageCenter>
    template <typename TopicType>
    void BufferMessagesFunctor<TMessageCenter>::operator()() {
        if (TopicType::destination != current_node) {
            TopicType message;
            message_center.get_message(message);

            uint8_t buffer[sizeof(TopicType)];
            TopicType::serialize(message, buffer);

            if (TopicType::destination == mc2::MessageNode::All) {
                // Send to all internode channels
                can_send_fifo.push(buffer, sizeof(TopicType));
                uart_send_fifo.push(buffer, sizeof(TopicType));
            } else {
                switch (TopicType::destination) {
                    case mc2::MessageNode::Chassis:
                        [[fallthrough]];
                    case mc2::MessageNode::Telemetry:
                        [[fallthrough]];
                    case mc2::MessageNode::Gimbal:
                        can_send_fifo.push(buffer, sizeof(TopicType));
                        break;
                    case mc2::MessageNode::MiniPC:
                        uart_send_fifo.push(buffer, sizeof(TopicType));
                        break;
                }
            }
        }
    }

    template <typename MessageMeta>
    bool MessageFIFO<MessageMeta>::push(const uint8_t* data, size_t data_length,
                                        const MessageMeta& meta) {
        bool data_pushed = data_fifo.push(data, data_length);
        bool meta_pushed = meta_buffer.push(meta);
        return data_pushed && meta_pushed;
    }

    template <typename MessageMeta>
    bool MessageFIFO<MessageMeta>::pop(uint8_t* data, MessageMeta& meta) {
        bool meta_popped = meta_buffer.pop(meta);

        size_t data_length = 0;
        bool data_popped = data_fifo.pop(data, data_length);
        return meta_popped && data_popped;
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    Communication<TMessageCenter, max_messages_per_tick>::Communication(
        TMessageCenter& _message_center,
        can2_tp::v1::CAN2TP<MAX_SINGLE_MESSAGE_SIZE>& _can2_tp,
        uc_uart::UC_UART<MAX_SINGLE_MESSAGE_SIZE>& _uc_uart_ref,
        isr::can::CAN_ISR& can_isr, isr::uart::UART_ISR& uart_isr,
        mc2::MessageNode _current_node)
        : message_center(_message_center),
          can2_tp_ref(_can2_tp),
          uc_uart_ref(_uc_uart_ref),
          can_isr_ref(can_isr),
          uart_isr_ref(uart_isr),
          current_node(_current_node) {
        buffer_message_functor.message_center = message_center;
        buffer_message_functor.can_send_fifo = can_send_fifo;
        buffer_message_functor.uart_send_fifo = uart_send_fifo;
        buffer_message_functor.current_node = current_node;
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    bool Communication<TMessageCenter, max_messages_per_tick>::init() {
        ASSERT(can_isr_ref.register_routine(
                   isr::can::ECallbacks::MESSAGE_PENDING,
                   [this](MW_CAN::ICAN& can, MW_CAN::BUS bus) {
                       this->can_isr_message_pending(can, bus);
                   }),
               "Failed to register CAN ISR routine.");
        ASSERT(
            uart_isr_ref.register_routine(
                isr::uart::ECallbacks::RECEIVE_COMPLETE,
                [this](MW_UART::IUART& uart, MW_UART::Peripheral peripheral) {
                    this->uart_isr_receive_complete(uart, peripheral);
                }),
            "Failed to register UART ISR routine.");
        ASSERT(uart_isr_ref.register_init([this](MW_UART::IUART& uart) {
            return this->uart_isr_init(uart);
        }),
               "Failed to register UART ISR init function.");
        return true;
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void Communication<TMessageCenter, max_messages_per_tick>::
        can_isr_message_pending(MW_CAN::BUS bus, isr::can::CANFrame frame) {
        if (bus == MW_CAN::BUS::CAN_2B) {}
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void Communication<TMessageCenter, max_messages_per_tick>::
        uart_isr_receive_complete(MW_UART::IUART& uart,
                                  MW_UART::Peripheral peripheral) {}

    template <typename TMessageCenter, size_t max_messages_per_tick>
    bool Communication<TMessageCenter, max_messages_per_tick>::uart_isr_init(
        MW_UART::IUART& uart) {
        uart.receive_data(MW_UART::Peripheral::UART1, uart_rx_buffer, 1);
        return true;
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void Communication<TMessageCenter,
                       max_messages_per_tick>::buffer_internode_messages() {
        mc2::interboard_foreach<TMessageCenter::Topics>(
            buffer_message_functor,
            mc2::get_tuple_index<TMessageCenter::Topics>());
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void
    Communication<TMessageCenter, max_messages_per_tick>::tick_send_messages() {
        // mc2::interboard_foreach(send_functor,
        //                         mc2::get_tuple_index<TMessageCenter::Topics>());

        can2_tp::v1::Meta can_meta;
        uint8_t can_data[MAX_SINGLE_MESSAGE_SIZE];

        uc_uart::DataFrameMeta uart_meta;
        uint8_t uart_data[MAX_SINGLE_MESSAGE_SIZE];
        for (int i = 0; i < max_messages_per_tick; ++i) {
            can2_tp::v1::CAN2BFrame next_frame;

            if (can2_tp_ref.get_next_send_fragment(next_frame)) {
                can
            } else {
            }
            bool has_can_message = can_send_fifo.pop(can_data, can_meta);
            if (has_can_message) {
                can2_tp_ref.send_message(
                    MW_CAN::BUS::CAN_2B, can_meta.destination_address,
                    can_meta.message_id, can_data, can_meta.length, 100);
            }

            bool has_uart_message = uart_send_fifo.pop(uart_data, uart_meta);
            if (has_uart_message) {
                uc_uart_ref.set_send_data(uart_meta.destination_address,
                                          uart_meta.message_id, uart_data,
                                          uart_meta.length);
            }
        }
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void Communication<TMessageCenter,
                       max_messages_per_tick>::tick_process_receive_messages() {
    }
};  // namespace comm
#endif
