#ifndef __COMMUNICATION_IPP
#define __COMMUNICATION_IPP

#include <type_traits>
#include "communication.hpp"

namespace comm {
    template <typename TMessageCenter, size_t max_messages_per_tick>
    Communication<TMessageCenter, max_messages_per_tick>::Communication(
        TMessageCenter& _message_center, can2_tp::v1::CAN2TP<128>& _can2_tp,
        uc_uart::UC_UART<256>& _uc_uart_ref, isr::can::CAN_ISR& can_isr,
        isr::uart::UART_ISR& uart_isr)
        : message_center(_message_center) {
        can_isr.register_routine(isr::can::ECallbacks::MESSAGE_PENDING,
                                 [this](MW_CAN::ICAN& can, MW_CAN::BUS bus) {
                                     this->can_isr_message_pending(can, bus);
                                 });
    }

    template <typename TMessageCenter, size_t max_messages_per_tick>
    void Communication<TMessageCenter, max_messages_per_tick>::
        can_isr_message_pending(MW_CAN::BUS bus, isr::can::CANFrame frame) {
        if (bus == MW_CAN::BUS::CAN_2) {}
    }
};  // namespace comm
#endif
