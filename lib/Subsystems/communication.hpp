#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "../Algorithms/fifo.hpp"
#include "../ISRs/can_isr.hpp"
#include "../ISRs/uart_isr.hpp"
#include "../Middleware/middleware_interfaces.hpp"
#include "../Modules/can2_tp.hpp"
#include "../Modules/uc_uart.hpp"

// TODO: Implement communication system.
// TODO: Integrate communication system into robot.

namespace comm {
    template <typename TMessageCenter, size_t max_messages_per_tick>
    class Communication {
       private:
        TMessageCenter& message_center;
        dsa::VarFIFO<1024, 20> can_send_fifo, uart_send_fifo;
        dsa::VarFIFO<1024, 20> can_receive_fifo, uart_receive_fifo;
        can2_tp::v1::CAN2TP<128>& can2_tp_ref;
        uc_uart::UC_UART<256>& uc_uart_ref;

       public:
        explicit Communication(TMessageCenter& _message_center,
                               can2_tp::v1::CAN2TP<128>& _can2_tp,
                               uc_uart::UC_UART<256>& _uc_uart_ref,
                               isr::can::CAN_ISR& can_isr,
                               isr::uart::UART_ISR& uart_isr);
        void init();

        void can_isr_message_pending(MW_CAN::BUS bus, isr::can::CANFrame frame);
        void uart_isr_receive_complete(MW_UART::IUART& uart,
                                       MW_UART::Peripheral peripheral);
        bool uart_isr_init(MW_UART::IUART& uart);

        /**
         * @brief Checks for and buffers messages from internode topics.
         */
        void buffer_internode_messages();

        /**
         * @brief Sends as many buffered messages as possible over all channels.
         */
        void tick_send_messages();
    };
}  // namespace comm

#ifndef __COMMUNICATION_IPP
#include "communication.ipp"
#endif

#endif