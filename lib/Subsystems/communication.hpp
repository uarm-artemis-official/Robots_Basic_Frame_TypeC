#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "can2_tp.hpp"
#include "can_isr.hpp"
#include "middleware_interfaces.hpp"
#include "uart_isr.hpp"
#include "uc_uart.hpp"


namespace comm {
    template <typename TMessageCenter>
    class Communication {
       private:
        TMessageCenter& message_center;

       public:
        explicit Communication(TMessageCenter& _message_center,
                               isr::can::CAN_ISR& can_isr);
        void init();
        void on_can_message_pending(MW_CAN::ICAN& can, MW_CAN::BUS bus);
        void on_uart_receive_complete(MW_UART::IUART& uart,
                                      MW_UART::Peripheral peripheral);
        void process_internode_messages();
    };
}  // namespace comm

#ifndef __COMMUNICATION_IPP
#include "communication.ipp"
#endif

#endif