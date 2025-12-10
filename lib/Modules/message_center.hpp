#ifndef __MESSAGE_CENTER_HPP
#define __MESSAGE_CENTER_HPP

#include <array>
#include <concepts>
#include <cstdint>
#include <cstring>
#include <functional>
#include <optional>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>
#include "../uarm_lib.hpp"
#include "middleware_interfaces.hpp"

namespace mc2 {
    inline namespace v2 {
        // Below are templates for defining message topics. Topics are generally
        // defined in a topics.hpp file specific to system. See robots/topics.hpp
        // for an example implementation.

        // Regular topics (i.e. Message Topics) are used for internal
        // communication within a single microcontroller. While Interboard
        // Message Topics are used for communication between microcontrollers
        // over CAN2. These templates are based on the concepts for MessageTopic
        // and InterboardMessageTopic defined later in this file. If there are
        // any discrepancies between the concepts and the actual templates, please
        // refer to the concepts and update the templates accordingly.

        /* Message Topic struct template.
        struct _ {
            static constexpr size_t queue_size = _;

            // Message fields...
        }
        */

        /* Interboard Message Topic struct template.
        struct _ {
            static constexpr MessageNode destination = MessageNode::_;
            static constexpr size_t serialized_size = _;
            static constexpr size_t queue_size = _;

            // Message fields...

            static void serialize(const _& msg,
                                std::span<uint8_t, serialized_size> dst) {
                (void) msg;
                (void) dst;
            }

            static void deserialize(_& msg,
                                    std::span<const uint8_t, serialized_size> src) {
                (void) msg;
                (void) src;
            }
        }
        */

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

        template <typename TopicRegistry>
        constexpr size_t registry_size_v = std::tuple_size_v<TopicRegistry>;

        template <typename TopicRegistry, size_t index>
        using TopicTypeAtIndex = std::tuple_element_t<index, TopicRegistry>;

        template <typename TopicRegistry, typename Topic, size_t index>
        consteval uint8_t get_topic_index_impl() {
            using TopicType = std::tuple_element_t<index, TopicRegistry>;
            if constexpr (std::is_same_v<TopicType, Topic>) {
                return index;
            } else {
                return registry_size_v<TopicRegistry>;
            }
        }

        template <typename TopicRegistry, typename Topic>
        consteval uint8_t get_topic_index() {
            constexpr size_t registry_size = registry_size_v<TopicRegistry>;
            auto arr = [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<uint8_t, registry_size> {
                    get_topic_index_impl<TopicRegistry, Topic, Is>()...};
            }(std::make_index_sequence<registry_size> {});

            size_t matches = 0;
            uint8_t last_index = registry_size;
            for (size_t i = 0; i < arr.size(); i++) {
                if (arr[i] != registry_size) {
                    matches++;
                    last_index = arr[i];
                }
            }

            if (matches == 1) {
                return last_index;
            } else {
                return registry_size;
            }
        }

        template <typename Tuple>
        constexpr auto get_tuple_index() {
            return std::make_index_sequence<std::tuple_size_v<Tuple>> {};
        }

        template <typename T, typename List, int index>
        struct type_present_operator {
            constexpr bool operator()() {
                return std::is_same_v<T, std::tuple_element_t<index, List>>;
            }
        };

        template <typename T, typename = void>
        constexpr bool is_interboard_message = false;

        // TODO: Add more robust check for InterboardMessage inheritance?
        template <typename T>
        constexpr bool
            is_interboard_message<T, std::void_t<decltype(T::Serializer)>> =
                true;

        template <typename List, int index>
        struct interboard_present_operator {
            constexpr bool operator()() {
                using TypeAtIndex = std::tuple_element_t<index, List>;
                return is_interboard_message<TypeAtIndex>;
            }
        };

        template <typename T, typename List, size_t... Is>
        constexpr auto get_type_present_array(std::index_sequence<Is...>) {
            return std::array<bool, std::tuple_size_v<List>> {
                type_present_operator<T, List, Is> {}()...};
        }

        template <typename List, size_t... Is>
        constexpr auto get_interboard_present_array(
            std::index_sequence<Is...>) {
            return std::array<bool, std::tuple_size_v<List>> {
                is_interboard_message<std::tuple_element_t<Is, List>>...};
        }

        template <typename List, typename F, typename TFArgs, size_t... Is>
        constexpr void for_each_topic(F&& f, TFArgs args,
                                      std::index_sequence<Is...>) {
            (f.template operator()<std::tuple_element_t<Is, List>>(args), ...);
        }

        template <int size>
        constexpr int count_true(std::array<bool, size> arr) {
            int count = 0;
            for (bool x : arr) {
                if (x)
                    count++;
            }
            return count;
        }

        template <int index, typename List, size_t... Is>
        constexpr bool is_unique(std::index_sequence<Is...>) {
            std::array<bool, std::tuple_size_v<List>> arr =
                get_type_present_array<std::tuple_element_t<index, List>, List>(
                    get_tuple_index<List>());
            return count_true<arr.size()>(arr) == 1;
        }

        template <typename List, size_t... Is>
        constexpr bool all_unique_types(std::index_sequence<Is...>) {
            std::array<bool, std::tuple_size_v<List>> arr {
                is_unique<Is, List>(get_tuple_index<List>())...};
            return count_true<arr.size()>(arr) == arr.size();
        }

        // TODO: Remove?
        template <typename T, typename TopicRegistry>
        constexpr size_t get_comm_id() {
            constexpr size_t index = get_topic_index<TopicRegistry, T>();
            return index + 100;
        }

        template <int index, typename List>
        constexpr size_t get_topic_type_size() {
            return sizeof(std::tuple_element_t<index, List>);
        }

        template <int index, typename List>
        constexpr size_t get_topic_queue_size() {
            return std::tuple_element_t<index, List>::queue_size;
        }

        template <int index, typename TopicRegistry>
        TopicHandle generate_topic_handle(MW_RTOS::IRTOS& rtos) {
            size_t queue_length = get_topic_queue_size<index, TopicRegistry>();
            size_t item_size = get_topic_type_size<index, TopicRegistry>();
            TopicHandle topic;
            rtos.queue_create(topic.queue, queue_length, item_size);
            topic.timestamps = std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> {};
            topic.recent_timestamp_index = 0;
            return topic;
        }

        /**
     * @brief Generate an array of TopicHandles for all topics in the TopicRegistry.
     * This function creates a TopicHandle for each topic registered in TopicRegistry.
     * 
     * TODO: Replace with consteval function that generates queue info at compile-time.
     *       The RTOS queues can then be created at runtime using simpler logic.
     * 
     * @param TopicRegistry A tuple type containing all registered topic types.
     * @param rtos Reference to the RTOS interface for queue creation.
     * @return std::array<TopicHandle, registry_size> Array of TopicHandles for each topic.
     */
        template <typename TopicRegistry>
        auto generate_topic_handles_array(MW_RTOS::IRTOS& rtos) {
            constexpr size_t registry_size = std::tuple_size_v<TopicRegistry>;
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<TopicHandle, registry_size> {
                    generate_topic_handle<Is, TopicRegistry>(rtos)...};
            }(std::make_index_sequence<registry_size> {});
        }

        template <typename TopicRegistry, typename Op, int index>
        void interboard_foreach_impl(Op& op) {
            using TopicType = std::tuple_element_t<index, TopicRegistry>;
            if constexpr (is_interboard_message<TopicType>) {
                op.template operator()<TopicType>();
            }
        }

        template <typename TopicRegistry, typename Op, size_t... Is>
        auto interboard_foreach(Op&& op, std::index_sequence<Is...>) {
            (interboard_foreach_impl<TopicRegistry, Op, Is>(op), ...);
        }

        template <typename Registry, size_t index>
        consteval auto generate_deserialize_directory_impl() {
            using T = std::tuple_element_t<index, Registry>;
            static_assert(std::is_trivially_copyable_v<T>);
            return [&](std::span<std::byte> dst, std::span<std::byte> src) {
                T msg;
                T::deserialize(msg, src.first<sizeof(T)>());
                std::memcpy(dst.data(), &msg, sizeof(T));
                return false;
            };
        }

        using IndexedDeserializer =
            std::function<bool(std::span<std::byte>, std::span<std::byte>)>;

        template <typename Registry>
        constexpr auto generate_deserialize_directory() {
            constexpr size_t registry_size = std::tuple_size_v<Registry>;
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<IndexedDeserializer, registry_size> {
                    generate_deserialize_directory_impl<Registry, Is>()...};
            }(std::make_index_sequence<registry_size> {});
        }

        template <typename T>
        concept MessageTopic = requires {
            T::queue_size;
        };

        template <typename T>
        concept InterboardMessageTopic =
            requires(T & msg, std::span<uint8_t, T::serialized_size> dst,
                     std::span<const uint8_t, T::serialized_size> src) {
            requires MessageTopic<T>;
            requires std::same_as<decltype(T::destination), mc2::MessageNode>;
            requires std::same_as<decltype(T::serialized_size), const size_t>;
            requires std::invocable<decltype(T::serialize), const T&,
                                    std::span<uint8_t, T::serialized_size>>;
            { T::serialize(msg, dst) } -> std::same_as<bool>;
            requires std::invocable<decltype(T::deserialize), T&,
                                    std::span<uint8_t, T::serialized_size>>;
            { T::deserialize(msg, src) } -> std::same_as<bool>;
        };

        template <MessageTopic... Topics>
        using create_topic_registry_t = std::tuple<Topics...>;

        template <typename TopicRegistry>
        class MC2 {
           private:
            std::array<TopicHandle, std::tuple_size_v<TopicRegistry>>
                topic_handles;
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
            bool init() {
                static_assert(all_unique_types<TopicRegistry>(
                                  get_tuple_index<TopicRegistry>()),
                              "Only unique topics within TopicRegistry");
                topic_handles =
                    generate_topic_handles_array<TopicRegistry>(rtos);
                // TODO: Finish
                return true;
            }

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
                T& message, MW_RTOS::TickType ticks_to_wait = 0) {
                TopicHandle& topic_handle =
                    topic_handles.at(get_topic_index<TopicRegistry, T>());
                MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
                ASSERT(
                    topic_queue != nullptr,
                    "Cannot get message from message_center for NULL pointer.");
                bool result = rtos.queue_get(
                    topic_queue, static_cast<void*>(&message), ticks_to_wait);
                if (result) {
                    MW_RTOS::TickType recent_message_timestamp =
                        topic_handle.timestamps.at(
                            topic_handle.recent_timestamp_index);
                    topic_handle.recent_timestamp_index =
                        (topic_handle.recent_timestamp_index - 1 +
                         MAX_TOPIC_QUEUE_SIZE) %
                        MAX_TOPIC_QUEUE_SIZE;
                    return std::make_optional(recent_message_timestamp);
                } else {
                    return {};
                }
            }

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
                T& message, MW_RTOS::TickType ticks_to_wait = 0) {
                TopicHandle& topic_handle =
                    topic_handles.at(get_topic_index<TopicRegistry, T>());
                MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
                ASSERT(
                    topic_queue != nullptr,
                    "Cannot get message from message_center for NULL pointer.");
                bool result = rtos.queue_peek(
                    topic_queue, static_cast<void*>(&message), ticks_to_wait);
                if (result) {
                    MW_RTOS::TickType recent_message_timestamp =
                        topic_handle.timestamps.at(
                            topic_handle.recent_timestamp_index);
                    return std::make_optional(recent_message_timestamp);
                } else {
                    return {};
                }
            }

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
                T& message, MW_RTOS::TickType ticks_to_wait = 0) {
                TopicHandle& topic_handle =
                    topic_handles.at(get_topic_index<TopicRegistry, T>());
                MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
                ASSERT(
                    topic_queue != nullptr,
                    "Cannot get message from message_center for NULL pointer.");
                bool result;
                if (get_topic_queue_size<get_topic_index<TopicRegistry, T>(),
                                         TopicRegistry>() == 1) {
                    result = rtos.queue_overwrite(topic_queue,
                                                  static_cast<void*>(&message));
                } else {
                    result = rtos.queue_pushback(topic_queue,
                                                 static_cast<void*>(&message),
                                                 ticks_to_wait);
                }
                if (result) {
                    MW_RTOS::TickType recent_tick = rtos.get_current_tick();
                    topic_handle.recent_timestamp_index =
                        (topic_handle.recent_timestamp_index + 1) %
                        topic_handle.timestamps.size();
                    topic_handle.timestamps.at(
                        topic_handle.recent_timestamp_index) = recent_tick;
                    return std::make_optional(recent_tick);
                } else {
                    return {};
                }
            }

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
                T& message, bool* will_context_switch = nullptr) {
                TopicHandle& topic_handle =
                    topic_handles.at(get_topic_index<TopicRegistry, T>());
                MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
                ASSERT(
                    topic_queue != nullptr,
                    "Cannot get message from message_center for NULL pointer.");
                bool result;
                if (get_topic_queue_size<get_topic_index<TopicRegistry, T>(),
                                         TopicRegistry>() == 1) {
                    result = rtos.queue_overwrite_from_isr(
                        topic_queue, static_cast<void*>(&message),
                        will_context_switch);
                } else {
                    result = rtos.queue_pushback_from_isr(
                        topic_queue, static_cast<void*>(&message),
                        will_context_switch);
                }

                if (result) {
                    MW_RTOS::TickType recent_tick = rtos.get_current_tick();
                    topic_handle.recent_timestamp_index =
                        (topic_handle.recent_timestamp_index + 1) %
                        topic_handle.timestamps.size();
                    topic_handle.timestamps.at(
                        topic_handle.recent_timestamp_index) = recent_tick;
                    return std::make_optional(recent_tick);
                } else {
                    return {};
                }
            }
        };
    }  // namespace v2
}  // namespace mc2

#endif