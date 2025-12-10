#ifndef __MESSAGE_CENTER_HPP
#define __MESSAGE_CENTER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include "middleware_interfaces.hpp"

namespace mc2 {
    constexpr size_t MAX_TOPIC_QUEUE_SIZE = 20;

    enum class MessageNode { Telemetry = 1, Chassis, Gimbal, MiniPC, All };

    template <size_t _queue_size>
    struct Topic {
        static_assert(_queue_size <= MAX_TOPIC_QUEUE_SIZE);
        static constexpr size_t queue_size = _queue_size;
    };

    // TODO: Add parameter to set size of message.
    template <size_t queue_size, size_t _serialized_size,
              MessageNode _destination, bool _also_local = false>
    struct InterboardMessage : Topic<queue_size> {
        static constexpr MessageNode destination = _destination;
        static constexpr bool also_local = _also_local;
        static constexpr size_t serialized_size = _serialized_size;
    };

    struct TopicHandle {
        MW_RTOS::QueueHandle queue;
        std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> timestamps;
        size_t recent_timestamp_index;
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

        /**
         * @brief Initialize message center internals for tracking activity for topics in TopicRegistry.
         * 
         * This function creates FIFO queues for each topic based on their corresponding topic message structs.
         * 
         * @return true if initialization succeeded, false otherwise.
         */
        bool init();

        /**
         * @brief Get a message of type T from its corresponding topic queue.
         * 
         * The oldest message is retrieved, removed from the queue, and set to the message reference.
         * 
         * @tparam T Message type to retrieve.
         * @param message Reference to store the retrieved message.
         * @param ticks_to_wait Maximum ticks to wait for a message.
         * @return Optional containing the tick count when the message was published, or nullopt if no message was available.
         */
        template <typename T>
        std::optional<MW_RTOS::TickType> get_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        /**
         * @brief Peek at the next message of type T in its corresponding topic queue without removing it.
         * 
         * This is a read-only operation; the message remains in the queue.
         * 
         * @tparam T Message type to peek at.
         * @param message Reference to store the peeked message.
         * @param ticks_to_wait Maximum ticks to wait for a message.
         * @return Optional containing the tick count when the message was peeked, or nullopt if no message was available.
         */
        template <typename T>
        std::optional<MW_RTOS::TickType> peek_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        /**
         * @brief Publish a message of type T to its corresponding topic queue.
         * 
         * The message is added to the end of the queue. If the queue is full, the behavior depends on the topic's queue size configuration.
         * If the queue size is 1, the existing message is overwritten. Otherwise, the function waits up to ticks_to_wait ticks for space to become available.
         * 
         * @tparam T Message type to publish.
         * @param message Reference to the message to publish.
         * @param ticks_to_wait Maximum ticks to wait if the queue is full.
         * @return Optional containing the tick count when the message was published, or nullopt if the message could not be published.
         */
        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message(
            T& message, MW_RTOS::TickType ticks_to_wait = 0);

        /**
         * @brief Publish a message of type T to its corresponding topic queue from an ISR context.
         * 
         * This function has the same behavior as pub_message, but is safe to call from an ISR.
         * 
         * @tparam T Message type to publish.
         * @param message Reference to the message to publish.
         * @param will_context_switch Pointer to a boolean that will be set to true if a context switch is required after publishing.
         * @return Optional containing the tick count when the message was published, or nullopt if the message could not be published.
         */
        template <typename T>
        std::optional<MW_RTOS::TickType> pub_message_from_isr(
            T& message, bool* will_context_switch = nullptr);
    };
}  // namespace mc2

#ifndef __MESSAGE_CENTER_IPP
#include "message_center.ipp"
#endif

#endif