#ifndef __COMMUNICATION_HPP
#define __COMMUNICATION_HPP

#include "can_isr.hpp"
#include "isotp.hpp"
#include "topics.hpp"
#include "uart_isr.hpp"
#include "uc_uart_interface.hpp"

namespace comm {
    template <typename FCanSend, typename FUartSend, typename FDelay>
    class Communication {
       private:
        mc2::RobotMC& mc;
        isr::can::CAN_ISR& can_isr;
        isr::uart::UART_ISR& uart_isr;
        uc_uart::v1::UC_UARTV1& uc_uart;
        isotp::ISOTP<FUartSend, FDelay> uart_isotp;
        isotp::ISOTP<FCanSend, FDelay> can_isotp;

       public:
        Communication(mc2::RobotMC& mc_ref, isr::can::CAN_ISR& can_ref,
                      isr::uart::UART_ISR& uart_ref);
        void init();
        void on_can_message_pending(MW_CAN::ICAN& can, MW_CAN::BUS bus);
        void on_uart_receive_complete(MW_UART::IUART& uart,
                                      MW_UART::Peripheral peripheral);
    };
}  // namespace comm

#include "communication.ipp"

#endif