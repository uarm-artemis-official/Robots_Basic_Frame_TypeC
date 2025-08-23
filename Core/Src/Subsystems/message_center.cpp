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

    void GimbalCommand::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT(-2 * PI < yaw && yaw < 2 * PI,
               "Outgoing yaw out of acceptable range (-2PI, 2PI).");
        ASSERT(-2 * PI < pitch && pitch < 2 * PI,
               "Outgoing pitch out of acceptable range (-2PI, 2PI).");
        ASSERT((command_bits & 0xffff0000) == 0,
               "Outgoing command_bits must not have 8 MSB set.");
        int16_t encoded_yaw = yaw * 5000;
        int16_t encoded_pitch = pitch * 5000;
        uint16_t encoded_command_bits = command_bits & 0xffff;
        memcpy(bytes.data(), &encoded_yaw, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t) + sizeof(int16_t),
               &encoded_command_bits, sizeof(uint16_t));
    }

    void GimbalCommand::decode(std::array<uint8_t, 200>& bytes) {
        int16_t encoded_yaw;
        int16_t encoded_pitch;
        uint16_t encoded_command_bits;
        memcpy(&encoded_yaw, bytes.data(), sizeof(int16_t));
        memcpy(&encoded_pitch, bytes.data() + sizeof(int16_t), sizeof(int16_t));
        memcpy(&encoded_command_bits,
               bytes.data() + sizeof(int16_t) + sizeof(int16_t),
               sizeof(uint16_t));
        yaw = static_cast<float>(encoded_yaw) / 5000;
        pitch = static_cast<float>(encoded_pitch) / 5000;
        command_bits = static_cast<uint32_t>(encoded_command_bits);
        ASSERT(-2 * PI < yaw && yaw < 2 * PI,
               "Incoming yaw out of acceptable range (-2PI, 2PI).");
        ASSERT(-2 * PI < pitch && pitch < 2 * PI,
               "Incoming pitch out of acceptable range (-2PI, 2PI).");
        ASSERT((command_bits & 0xffff0000) == 0,
               "Incoming command_bits must not have 8 MSB set.");
    }

    void ShootCommand::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT((extra_bits & 0xffff0000) == 0,
               "Incoming extra_bits must not have 16 MSB set.");
        uint32_t encoded_command_bits = command_bits;
        uint16_t encoded_extra_bits = extra_bits && 0xffff;
        memcpy(bytes.data(), &encoded_command_bits, sizeof(uint32_t));
        memcpy(bytes.data() + sizeof(uint32_t), &encoded_extra_bits,
               sizeof(uint16_t));
    }

    void ShootCommand::decode(std::array<uint8_t, 200>& bytes) {
        uint32_t encoded_command_bits;
        uint16_t encoded_extra_bits;
        memcpy(&encoded_command_bits, bytes.data(), sizeof(uint32_t));
        memcpy(&encoded_extra_bits, bytes.data(), sizeof(uint16_t));
        command_bits = encoded_command_bits;
        extra_bits = encoded_extra_bits & 0xffff;
        ASSERT((extra_bits & 0xffff0000) == 0,
               "Incoming extra_bits must not have 16 MSB set.");
    }

    void GimbalRelativeAngles::encode(std::array<uint8_t, 200>& bytes) {
        ASSERT(-PI < yaw && yaw < PI,
               "Outgoing yaw out of acceptable range (-PI, PI).");
        ASSERT(-PI < pitch && pitch < PI,
               "Outgoing pitch out of acceptable range (-PI, PI).");
        int16_t encoded_yaw = yaw * 10000;
        int16_t encoded_pitch = pitch * 10000;
        memcpy(bytes.data(), &encoded_yaw, sizeof(int16_t));
        memcpy(bytes.data() + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
    }

    void GimbalRelativeAngles::decode(std::array<uint8_t, 200>& bytes) {
        int16_t encoded_yaw;
        int16_t encoded_pitch;
        memcpy(&encoded_yaw, bytes.data(), sizeof(int16_t));
        memcpy(&encoded_pitch, bytes.data() + sizeof(int16_t), sizeof(int16_t));
        yaw = static_cast<float>(encoded_yaw) / 10000;
        pitch = static_cast<float>(encoded_pitch) / 10000;
        ASSERT(-PI < yaw && yaw < PI,
               "Incoming yaw out of acceptable range (-PI, PI).");
        ASSERT(-PI < pitch && pitch < PI,
               "Incoming pitch out of acceptable range (-PI, PI).");
    }

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
            std::array<uint32_t, MAX_TOPIC_QUEUE_SIZE> {}, 0};
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
        TopicHandle topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        QueueHandle_t topic_queue = topic_handle.queue;
        ASSERT(topic_queue != NULL,
               "Cannot get message from message_center for NULL pointer.");
        BaseType_t result = xQueueReceive(topic_queue, this, ticks_to_wait);
        if (result == pdTrue) {
            uint32_t latest_message_timestamp =
                topic_handle.timestamps.at(topic_handle.latest_timestamp_index);
            topic_handle.latest_timestamp_index =
                (topic_handle.latest_timestamp_index - 1 +
                 MAX_TOPIC_QUEUE_SIZE) %
                MAX_TOPIC_QUEUE_SIZE;
            return latest_message_timestamp;
        } else {
            return 0;
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    uint32_t MC2<TopicRegistry>::peek_message(T& message,
                                              uint32_t ticks_to_wait) {
        TopicHandle topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        QueueHandle_t topic_queue = topic_handle.queue;
        ASSERT(topic_queue != NULL,
               "Cannot get message from message_center for NULL pointer.");
        BaseType_t result = xQueuePeek(topic_queue, this, ticks_to_wait);
        if (result == pdTrue) {
            uint32_t latest_message_timestamp =
                topic_handle.timestamps.at(topic_handle.latest_timestamp_index);
            return latest_message_timestamp;
        } else {
            return 0;
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    void MC2<TopicRegistry>::pub_message(T& message) {
        TopicHandle topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        QueueHandle_t topic_queue = topic_handle.queue;
        ASSERT(topic_queue != NULL,
               "Cannot get message from message_center for NULL pointer.");
        BaseType_t result;
        if (get_topic_queue_size<get_index<T, TopicRegistry>(),
                                 TopicRegistry>() == 1) {
            result = xQueueOverwrite(topic_queue, &message);
        } else {
            result = xQueueSendToBack(topic_queue, &message, 0);
        }
        if (result == pdTrue) {
            uint32_t latest_tick = uwTick;
            topic_handle.latest_timestamp_index =
                (topic_handle.latest_timestamp_index + 1) %
                topic_handle.timestamps.size();
            topic_handle.timestamps.at(topic_handle.latest_timestamp_index) =
                latest_tick;
            return latest_tick;
        } else {
            return 0;
        }
    }

    template <typename TopicRegistry>
    template <typename T>
    void MC2<TopicRegistry>::pub_message_from_isr(
        T& message, uint8_t* will_context_switch) {
        TopicHandle topic_handle =
            topic_handles.at(get_index<T, TopicRegistry>());
        QueueHandle_t topic_queue = topic_handle.queue;
        ASSERT(topic_queue != NULL,
               "Cannot get message from message_center for NULL pointer.");

        BaseType_t result;
        if (get_topic_queue_size<get_index<T, TopicRegistry>(),
                                 TopicRegistry>() == 1) {
            result = xQueueOverwriteFromISR(topic_handle.queue_handle, data_ptr,
                                            &context_switch);
        } else {
            result = xQueueSendToBackFromISR(topic_handle.queue_handle,
                                             data_ptr, &context_switch);
        }

        if (result == pdTrue) {
            uint32_t latest_tick = uwTick;
            topic_handle.latest_timestamp_index =
                (topic_handle.latest_timestamp_index + 1) %
                topic_handle.timestamps.size();
            topic_handle.timestamps.at(topic_handle.latest_timestamp_index) =
                latest_tick;
            return latest_tick;
        } else {
            return 0;
        }
    }
}  // namespace mc2