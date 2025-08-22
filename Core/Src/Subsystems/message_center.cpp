#include "message_center.hpp"
#include <utility>
#include "uarm_lib.hpp"
#include "uarm_os.hpp"

void MessageCenter::init() {
    if (!initialized) {
        for (size_t i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t);
             i++) {
            topic_handles[i].queue_handle = xQueueCreate(
                topic_handles[i].queue_length, topic_handles[i].item_size);
        }
        initialized = true;
    } else {
        ASSERT(false, "Attempt to initialize initialized message center.");
    }
}

MessageCenter& MessageCenter::get_instance() {
    static MessageCenter message_center;
    return message_center;
}

Topic_Handle_t& MessageCenter::get_topic_handle(Topic_Name_t name) {
    for (size_t i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t);
         i++) {
        if (topic_handles[i].name == name) {
            return topic_handles[i];
        }
    }
    ASSERT(false, "Cannot find topic by topic name in message center.");
}

// TODO: Support NULL data_ptr? Or add function to check if a topic has message.
uint8_t MessageCenter::get_message(Topic_Name_t topic, void* data_ptr,
                                   int ticks_to_wait) {
    ASSERT(data_ptr != NULL,
           "Cannot get message from message_center for NULL pointer.");
    if (!initialized)
        return 0;
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    return xQueueReceive(topic_handle.queue_handle, data_ptr, ticks_to_wait);
}

uint8_t MessageCenter::peek_message(Topic_Name_t topic, void* data_ptr,
                                    int ticks_to_wait) {
    ASSERT(data_ptr != NULL,
           "Cannot peek message from message_center for NULL pointer.");
    if (!initialized)
        return 0;
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    return xQueuePeek(topic_handle.queue_handle, data_ptr, ticks_to_wait);
}

uint8_t MessageCenter::pub_message(Topic_Name_t topic, void* data_ptr) {
    ASSERT(data_ptr != NULL,
           "Cannot publish NULL data pointer to message_center.");
    if (!initialized)
        return 0;
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    if (topic_handle.queue_length == 1) {
        return xQueueOverwrite(topic_handle.queue_handle, data_ptr);
    } else {
        return xQueueSendToBack(topic_handle.queue_handle, data_ptr, 0);
    }
}

uint8_t MessageCenter::pub_message_from_isr(Topic_Name_t topic, void* data_ptr,
                                            uint8_t* will_context_switch) {
    ASSERT(data_ptr != NULL,
           "Cannot publish NULL data pointer from ISR to message_center.");
    if (!initialized)
        return 0;
    BaseType_t context_switch, res;
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    if (topic_handle.queue_length == 1) {
        res = xQueueOverwriteFromISR(topic_handle.queue_handle, data_ptr,
                                     &context_switch);
    } else {
        res = xQueueSendToBackFromISR(topic_handle.queue_handle, data_ptr,
                                      &context_switch);
    }

    if (will_context_switch != NULL)
        *will_context_switch = (uint8_t) context_switch;
    return res;
}

namespace mc2 {
#include "uarm_os.hpp"

    template <typename Tuple>
    constexpr auto get_tuple_index() {
        return std::make_index_sequence<std::tuple_size_v<Tuple>> {};
    }

    template <typename T, typename List, int index>
    struct index_reducer {
        constexpr bool operator()() {
            return std::is_same_v<T, std::tuple_element_t<index, List>>;
        }
    };

    template <typename T, typename List, size_t... Is>
    constexpr auto get_type_present_array(std::index_sequence<Is...>) {
        return std::array<bool, std::tuple_size_v<List>> {
            index_reducer<T, List, Is> {}.template operator()()...
        };
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
        for (int i = 0; i < arr.size(); i++) {
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
        static_assert(index < std::tuple_size_v<List>);
        return index;
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
    TopicHandle generate_topic_handle() {
        return TopicHandle {
            xQueueCreate((get_topic_queue_size<index, TopicRegistry>()),
                         (get_topic_type_size<index, TopicRegistry>())),
            std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> {}};
    }

    template <typename TopicRegistry, size_t... Is>
    auto generate_topic_handles_array(std::index_sequence<Is...>) {
        return std::array<TopicHandle, std::tuple_size_v<TopicRegistry>> {
            generate_topic_handle<Is, TopicRegistry>()...};
    }

    template <typename TopicRegistry>
    void MC2<TopicRegistry>::init() {
        static_assert(
            all_unique_types<TopicRegistry>(get_tuple_index<TopicRegistry>()),
            "Only unique topics within TopicRegistry");
        topic_handles = generate_topic_handles_array<TopicRegistry>(
            get_tuple_index<TopicRegistry>());
    }

    template <typename TopicRegistry>
    template <typename T>
    uint32_t MC2<TopicRegistry>::get_message(T& message,
                                             uint32_t ticks_to_wait) {
        return 0;
    }

    template <typename TopicRegistry>
    template <typename T>
    uint32_t MC2<TopicRegistry>::peek_message(T& message,
                                              uint32_t ticks_to_wait) {
        return 0;
    }

    template <typename TopicRegistry>
    template <typename T>
    void MC2<TopicRegistry>::pub_message(T& message) {}

    template <typename TopicRegistry>
    template <typename T>
    void MC2<TopicRegistry>::pub_message_from_isr(
        T& message, uint8_t* will_context_switch) {}
}  // namespace mc2