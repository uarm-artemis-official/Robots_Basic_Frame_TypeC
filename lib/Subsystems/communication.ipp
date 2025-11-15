#ifndef __COMMUNICATION_IPP
#define __COMMUNICATION_IPP

#include <type_traits>
#include "communication.hpp"

namespace comm {
    template <typename TMessageCenter>
    Communication<TMessageCenter>::Communication(
        TMessageCenter& _message_center, isr::can::CAN_ISR& can_isr)
        : message_center(_message_center) {
        can_isr.register_routine(isr::can::ECallbacks::MESSAGE_PENDING,
                                 [this](MW_CAN::ICAN& can, MW_CAN::BUS bus) {
                                     this->on_can_message_pending(can, bus);
                                 });
    }
}  // namespace comm
#endif