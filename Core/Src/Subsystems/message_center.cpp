#include "message_center.h"
#include "uarm_lib.h"
#include "uarm_os.h"

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

uint8_t MessageCenter::get_message(Topic_Name_t topic, void* data_ptr,
                                   int ticks_to_wait) {
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    return xQueueReceive(topic_handle.queue_handle, data_ptr, ticks_to_wait);
}

uint8_t MessageCenter::peek_message(Topic_Name_t topic, void* data_ptr,
                                    int ticks_to_wait) {
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    return xQueuePeek(topic_handle.queue_handle, data_ptr, ticks_to_wait);
}

uint8_t MessageCenter::pub_message(Topic_Name_t topic, void* data_ptr) {
    Topic_Handle_t& topic_handle = get_topic_handle(topic);
    if (topic_handle.queue_length == 1) {
        return xQueueOverwrite(topic_handle.queue_handle, data_ptr);
    } else {
        return xQueueSendToBack(topic_handle.queue_handle, data_ptr, 0);
    }
}

uint8_t MessageCenter::pub_message_from_isr(Topic_Name_t topic, void* data_ptr,
                                            uint8_t* will_context_switch) {
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
    *will_context_switch = (uint8_t) context_switch;
    return res;
}
