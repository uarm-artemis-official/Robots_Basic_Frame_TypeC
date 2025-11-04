#ifndef __MESSAGE_CENTER_HPP
#define __MESSAGE_CENTER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include "middleware_interfaces.hpp"

namespace mc2 {
    constexpr size_t MAX_TOPIC_QUEUE_SIZE = 20;

    enum class MessageNode { Telemetry, Chassis, Gimbal, All };

    template <size_t _queue_size>
    struct Topic {
        static_assert(_queue_size <= MAX_TOPIC_QUEUE_SIZE);
        static constexpr size_t queue_size = _queue_size;
    };

    template <typename TMessage, MessageNode _destination,
              bool _also_local = false>
    struct InterboardMessage {
        static constexpr MessageNode destination = _destination;
        static constexpr bool also_local = _also_local;

        virtual void encode(std::array<uint8_t, 200>& bytes) = 0;
        virtual void decode(std::array<uint8_t, 200>& bytes) = 0;
    };

    struct TopicHandle {
        MW_RTOS::QueueHandle queue;
        std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> timestamps;
        size_t recent_timestamp_index;
    };

    struct TopicInfo {
        size_t queue_size;
        size_t item_size;
    };
}  // namespace mc2

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

#ifndef __MESSAGE_CENTER_IPP
#include "message_center.ipp"
#endif

#endif