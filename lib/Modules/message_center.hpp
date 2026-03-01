#ifndef __MESSAGE_CENTER_HPP
#define __MESSAGE_CENTER_HPP

#include <algorithm>
#include <array>
#include <concepts>
#include <cstddef>
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

// TODO: Add markers for more explicit declaration of topic types -> add better checking too.
// TODO: Add functionality for topics to be both local and interboard topics.
namespace mc2 {
    inline namespace v2 {
        // Below are templates for defining message topics. Topics are generally
        // defined in a topics.hpp file specific to system. See robots/topics.hpp
        // for an example implementation.

        // Regular topics (i.e. Message Topics) are used for internal
        // communication within a single microcontroller.

        /* Message Topic struct template.
        struct _ {
            static constexpr size_t queue_size = _;

            // Message fields...
        }
        */

        constexpr size_t MAX_TOPIC_QUEUE_SIZE = 20;
        constexpr size_t TOPIC_ID_OFFSET = 100;

        enum class MessageNode { Telemetry = 1, Chassis, Gimbal, MiniPC, All };

        struct TopicMeta {
            uint8_t topic_id;
            size_t queue_size;
            size_t item_size;
        };

        struct TopicHandle {
            MW_RTOS::QueueHandle queue;
            std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> timestamps;
            size_t recent_timestamp_index;
            TopicMeta meta;
        };

        template <typename T>
        concept MessageTopic = requires {
            requires std::same_as<decltype(T::queue_size), const size_t>;
            T::queue_size <= MAX_TOPIC_QUEUE_SIZE;
        };



        template <MessageTopic... Topics>
        using create_topic_registry_t = std::tuple<Topics...>;

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

        template <typename T, typename TopicRegistry>
        constexpr auto get_type_present_array() {
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<bool, std::tuple_size_v<TopicRegistry>> {
                    []() {
                        return std::is_same_v<
                            T, std::tuple_element_t<Is, TopicRegistry>>;
                    }()...};
            }(std::make_index_sequence<registry_size_v<TopicRegistry>> {});
        }

        template <int index, typename List>
        constexpr bool is_unique() {
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                std::array<bool, std::tuple_size_v<List>> arr =
                    get_type_present_array<std::tuple_element_t<index, List>,
                                           List>();
                return std::ranges::count(arr, true) == 1;
            }(std::make_index_sequence<std::tuple_size_v<List>> {});
        }

        template <typename List>
        constexpr bool all_unique_types() {
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                std::array<bool, std::tuple_size_v<List>> arr {
                    is_unique<Is, List>()...};

                return std::ranges::count(arr, true) ==
                       static_cast<int32_t>(arr.size());
            }(std::make_index_sequence<std::tuple_size_v<List>> {});
        }

        constexpr uint8_t get_topic_id_from_index(size_t index) {
            return static_cast<uint8_t>(TOPIC_ID_OFFSET + index);
        }

        // TODO: Remove?
        template <typename T, typename TopicRegistry>
        constexpr size_t get_comm_id() {
            constexpr size_t index = get_topic_index<TopicRegistry, T>();
            return get_topic_id_from_index(index);
        }

        constexpr size_t get_index_from_topic_id(uint8_t topic_id) {
            return static_cast<size_t>(topic_id) - TOPIC_ID_OFFSET;
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
            using Topic = std::tuple_element_t<index, TopicRegistry>;
            size_t queue_length = get_topic_queue_size<index, TopicRegistry>();
            size_t item_size = get_topic_type_size<index, TopicRegistry>();
            TopicHandle topic;
            rtos.queue_create(topic.queue, queue_length, item_size);
            topic.timestamps = std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> {};
            topic.recent_timestamp_index = 0;
            topic.meta.queue_size = Topic::queue_size;
            topic.meta.item_size = sizeof(Topic);
            topic.meta.topic_id = get_comm_id<Topic, TopicRegistry>();



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

        using IndexableDeserializer = std::function<bool(
            std::span<std::byte>, std::span<const std::byte>)>;

        using IndexableSerializer = std::function<bool(
            std::span<std::byte>, std::span<const std::byte>)>;

        template <typename TopicRegistry>
        consteval auto generate_topic_sizes() {
            constexpr size_t registry_size = std::tuple_size_v<TopicRegistry>;
            return [&]<size_t... Is>(std::index_sequence<Is...>) {
                return std::array<size_t, registry_size> {
                    get_topic_type_size<Is, TopicRegistry>()...};
            }(std::make_index_sequence<registry_size> {});
        }

        template <typename TopicRegistry>
        consteval size_t get_registry_max_message_size() {
            constexpr auto topic_sizes_array =
                generate_topic_sizes<TopicRegistry>();
            size_t max_size = 0;
            for (size_t size : topic_sizes_array) {
                if (size > max_size) {
                    max_size = size;
                }
            }
            return max_size;
        }

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
                static_assert(all_unique_types<TopicRegistry>(),
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

            std::optional<MW_RTOS::TickType> get_byte_message(
                std::span<std::byte> dst, size_t& message_size,
                uint8_t topic_id, MW_RTOS::TickType ticks_to_wait = 0) {
                size_t index = get_index_from_topic_id(topic_id);

                if (index >= registry_size_v<TopicRegistry>) {
                    return {};
                }

                TopicHandle& topic_handle = topic_handles[index];

                ASSERT(topic_handle.queue != nullptr,
                       "Cannot get message from message_center for NULL "
                       "pointer.");
                ASSERT(dst.size() >= topic_handle.meta.item_size,
                       "Destination buffer too small for message_center "
                       "byte message retrieval.");

                bool res = rtos.queue_get(topic_handle.queue, dst.data(),
                                          ticks_to_wait);
                if (res) {
                    message_size = topic_handle.meta.item_size;
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
                ASSERT(topic_queue != nullptr,
                       "Cannot get message from message_center for NULL "
                       "pointer.");
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
                ASSERT(topic_queue != nullptr,
                       "Cannot get message from message_center for NULL "
                       "pointer.");
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

            std::optional<MW_RTOS::TickType> pub_byte_message(
                std::span<const std::byte> src, uint8_t topic_id,
                MW_RTOS::TickType ticks_to_wait = 0) {
                size_t index = get_index_from_topic_id(topic_id);

                if (index >= registry_size_v<TopicRegistry>) {
                    return {};
                }

                TopicHandle& topic_handle = topic_handles[index];
                ASSERT(topic_handle.queue != nullptr,
                       "Cannot publish byte message to message_center for "
                       "NULL pointer.");
                ASSERT(src.size() == topic_handle.meta.item_size,
                       "Source span size does not match topic item size.");

                bool result;
                if (topic_handle.meta.queue_size == 1) {
                    result = rtos.queue_overwrite(
                        topic_handle.queue,
                        const_cast<void*>(
                            static_cast<const void*>(src.data())));
                } else {
                    result = rtos.queue_pushback(
                        topic_handle.queue,
                        const_cast<void*>(static_cast<const void*>(src.data())),
                        ticks_to_wait);
                }

                if (result) {
                    MW_RTOS::TickType recent_tick = rtos.get_current_tick();
                    topic_handle.recent_timestamp_index =
                        (topic_handle.recent_timestamp_index + 1) %
                        topic_handle.timestamps.size();
                    topic_handle
                        .timestamps[topic_handle.recent_timestamp_index] =
                        recent_tick;
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
                ASSERT(topic_queue != nullptr,
                       "Cannot get message from message_center for NULL "
                       "pointer.");
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

            /**
             * @brief Get the TopicMeta information for a given topic ID.
             * 
             * @param topic_id The ID of the topic to retrieve metadata for.
             * @return const TopicMeta& Reference to the TopicMeta structure.
             */
            const TopicMeta& get_topic_meta(uint8_t topic_id) {
                size_t index = get_index_from_topic_id(topic_id);

                ASSERT(index < registry_size_v<TopicRegistry>,
                       "Topic ID does not correspond to a valid topic.");

                return topic_handles[index].meta;
            }
        };
    }  // namespace v2
}  // namespace mc2

#endif