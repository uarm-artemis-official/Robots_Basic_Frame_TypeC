#ifndef __MESSAGE_CENTER_IPP
#define __MESSAGE_CENTER_IPP

#include <array>
#include <cstdint>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include "../uarm_lib.hpp"
#include "message_center.hpp"

namespace mc2 {
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
        is_interboard_message<T, std::void_t<decltype(T::Serializer)>> = true;

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
    constexpr auto get_interboard_present_array(std::index_sequence<Is...>) {
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

    template <int size>
    constexpr size_t index_first_true(std::array<bool, size> arr) {
        for (size_t i = 0; i < arr.size(); i++) {
            if (arr.at(i)) {
                return i;
            }
        }
        return arr.size();
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

    template <typename T, typename List, size_t... Is>
    constexpr size_t get_index_impl(std::index_sequence<Is...>) {
        std::array<bool, std::tuple_size_v<List>> arr =
            get_type_present_array<T, List>(get_tuple_index<List>());
        return index_first_true<arr.size()>(arr);
    }

    template <typename T, typename List>
    constexpr size_t get_index() {
        constexpr size_t index =
            get_index_impl<T, List>(get_tuple_index<List>());
        static_assert(index < std::tuple_size_v<List>,
                      "Cannot find index of topic");
        return index;
    }

    // TODO: Remove?
    template <typename T, typename List>
    constexpr size_t get_comm_id() {
        constexpr size_t index = get_index<T, List>();
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

    template <typename TopicRegistry, size_t... Is>
    auto generate_topic_handles_array(std::index_sequence<Is...>,
                                      MW_RTOS::IRTOS& rtos) {
        return std::array<TopicHandle, std::tuple_size_v<TopicRegistry>> {
            generate_topic_handle<Is, TopicRegistry>(rtos)...};
    }

    template <typename TopicRegistry, typename Op, int index>
    void interboard_foreach_impl(Op& op) {
        using TopicType = std::tuple_element_t<index, TopicRegistry>;
        if constexpr (is_interboard_message<TopicType>) {
            op.template operator()<TopicType>();
        }
    }

    template <typename TopicRegistry, typename Op, size_t... Is>
    auto interboard_foreach(Op& op, std::index_sequence<Is...>) {
        (interboard_foreach_impl<TopicRegistry, Op, Is>(op), ...);
    }

    template <typename TopicRegistry>
    bool MC2<TopicRegistry>::init() {
        static_assert(
            all_unique_types<TopicRegistry>(get_tuple_index<TopicRegistry>()),
            "Only unique topics within TopicRegistry");
        topic_handles = generate_topic_handles_array<TopicRegistry>(
            get_tuple_index<TopicRegistry>(), rtos);
        // TODO: Finish
        return true;
    }

    template <typename TopicRegistry>
    template <typename T>
    std::optional<MW_RTOS::TickType> MC2<TopicRegistry>::get_message(
        T& message, MW_RTOS::TickType ticks_to_wait) {
        TopicHandle& topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
        ASSERT(topic_queue != nullptr,
               "Cannot get message from message_center for NULL pointer.");
        bool result = rtos.queue_get(topic_queue, static_cast<void*>(&message),
                                     ticks_to_wait);
        if (result) {
            MW_RTOS::TickType recent_message_timestamp =
                topic_handle.timestamps.at(topic_handle.recent_timestamp_index);
            topic_handle.recent_timestamp_index =
                (topic_handle.recent_timestamp_index - 1 +
                 MAX_TOPIC_QUEUE_SIZE) %
                MAX_TOPIC_QUEUE_SIZE;
            return std::make_optional(recent_message_timestamp);
        } else {
            return {};
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    std::optional<MW_RTOS::TickType> MC2<TopicRegistry>::peek_message(
        T& message, MW_RTOS::TickType ticks_to_wait) {
        TopicHandle& topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
        ASSERT(topic_queue != nullptr,
               "Cannot get message from message_center for NULL pointer.");
        bool result = rtos.queue_peek(topic_queue, static_cast<void*>(&message),
                                      ticks_to_wait);
        if (result) {
            MW_RTOS::TickType recent_message_timestamp =
                topic_handle.timestamps.at(topic_handle.recent_timestamp_index);
            return std::make_optional(recent_message_timestamp);
        } else {
            return {};
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    std::optional<MW_RTOS::TickType> MC2<TopicRegistry>::pub_message(
        T& message, MW_RTOS::TickType ticks_to_wait) {
        TopicHandle& topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
        ASSERT(topic_queue != nullptr,
               "Cannot get message from message_center for NULL pointer.");
        bool result;
        if (get_topic_queue_size<get_index<T, TopicRegistry>(),
                                 TopicRegistry>() == 1) {
            result =
                rtos.queue_overwrite(topic_queue, static_cast<void*>(&message));
        } else {
            result = rtos.queue_pushback(
                topic_queue, static_cast<void*>(&message), ticks_to_wait);
        }
        if (result) {
            MW_RTOS::TickType recent_tick = rtos.get_current_tick();
            topic_handle.recent_timestamp_index =
                (topic_handle.recent_timestamp_index + 1) %
                topic_handle.timestamps.size();
            topic_handle.timestamps.at(topic_handle.recent_timestamp_index) =
                recent_tick;
            return std::make_optional(recent_tick);
        } else {
            return {};
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    std::optional<MW_RTOS::TickType> MC2<TopicRegistry>::pub_message_from_isr(
        T& message, bool* will_context_switch) {
        TopicHandle& topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        MW_RTOS::QueueHandle topic_queue = topic_handle.queue;
        ASSERT(topic_queue != nullptr,
               "Cannot get message from message_center for NULL pointer.");
        bool result;
        if (get_topic_queue_size<get_index<T, TopicRegistry>(),
                                 TopicRegistry>() == 1) {
            result = rtos.queue_overwrite_from_isr(
                topic_queue, static_cast<void*>(&message), will_context_switch);
        } else {
            result = rtos.queue_pushback_from_isr(
                topic_queue, static_cast<void*>(&message), will_context_switch);
        }

        if (result) {
            MW_RTOS::TickType recent_tick = rtos.get_current_tick();
            topic_handle.recent_timestamp_index =
                (topic_handle.recent_timestamp_index + 1) %
                topic_handle.timestamps.size();
            topic_handle.timestamps.at(topic_handle.recent_timestamp_index) =
                recent_tick;
            return std::make_optional(recent_tick);
        } else {
            return {};
        }
    }
}  // namespace mc2

#endif