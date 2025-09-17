#ifndef __SUBSYSTEMS_MODULES_HPP
#define __SUBSYSTEMS_MODULES_HPP

#include "middleware_interfaces.hpp"
#include "subsystems_types.hpp"

namespace mc2 {
    template <typename TopicRegistry>
    class MC2 {
       private:
        std::array<TopicHandle, std::tuple_size_v<TopicRegistry>> topic_handles;
        MW_RTOS::IRTOS& rtos;

       public:
        using Topics = TopicRegistry;

        MC2(MW_RTOS::IRTOS& _rtos) : rtos(_rtos) {}

        bool init();

        template <typename T>
        std::optional<MW_RTOS::TickType> get_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        template <typename T>
        std::optional<MW_RTOS::TickType> peek_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message_from_isr(
            T& message, bool* will_context_switch = nullptr);
    };
}  // namespace mc2

#include "message_center.ipp"

#endif