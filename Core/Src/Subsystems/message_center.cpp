#include "message_center.h"

// TODO: Add queue configurations for reading and writing permissions
// Motivation: simplify methods to pub_message and get_message only.
// TODO: Add message ID to distinguish messages on peek queues.
static Topic_Handle_t topic_handles[] = {
    Topic_Handle_t {MOTOR_SET, sizeof(MotorSetMessage_t), 5, NULL},
    Topic_Handle_t {MOTOR_READ, sizeof(MotorReadMessage_t), 1, NULL},
    Topic_Handle_t {RC_INFO, sizeof(RCInfoMessage_t), 1, NULL},
    Topic_Handle_t {COMM_OUT, sizeof(CANCommMessage_t), 5, NULL},
    Topic_Handle_t {COMM_IN, sizeof(MotorSetMessage_t), 5, NULL},
    Topic_Handle_t {IMU_READINGS, sizeof(float) * 2, 1, NULL},

    // [yaw, pitch]
    Topic_Handle_t {GIMBAL_REL_ANGLES, sizeof(float) * 2, 1, NULL},
    // TODO: Make struct with necessary info from ref system.
    Topic_Handle_t {REF_INFO, 0, 1, NULL},
    // TODO: Make struct with necessary info for commands from player inputs.
    Topic_Handle_t {PLAYER_COMMANDS, 0, 1, NULL},

    Topic_Handle_t {PLAYER_COMMANDS, 0, 1, NULL},
    Topic_Handle_t {REFEREE_INFO, sizeof(uint8_t) * 2, 1, NULL},
    Topic_Handle_t {RC_RAW, sizeof(uint8_t) * 18, 5, NULL},
    Topic_Handle_t {UC_PACK_IN, sizeof(uint8_t) * 64, 10, NULL},
    Topic_Handle_t {AUTO_AIM, sizeof(float) * 2, 1, NULL},
};

static Topic_Handle_t* get_topic_handle(Topic_Name_t name) {
    for (size_t i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t);
         i++) {
        if (topic_handles[i].name == name) {
            return &(topic_handles[i]);
        }
    }
    // TODO: Add error handling for this case.
    //	Error_Handler();
    return NULL;
}

void message_center_init() {
    for (size_t i = 0; i < sizeof(topic_handles) / sizeof(Topic_Handle_t);
         i++) {
        topic_handles[i].queue_handle = xQueueCreate(
            topic_handles[i].queue_length, topic_handles[i].item_size);
    }
}

uint8_t get_message(Topic_Name_t topic, void* data_ptr, int ticks_to_wait) {
    Topic_Handle_t* topic_handle = get_topic_handle(topic);
    return xQueueReceive(topic_handle->queue_handle, data_ptr, ticks_to_wait);
}

uint8_t peek_message(Topic_Name_t topic, void* data_ptr, int ticks_to_wait) {
    Topic_Handle_t* topic_handle = get_topic_handle(topic);
    return xQueuePeek(topic_handle->queue_handle, data_ptr, ticks_to_wait);
}

uint8_t pub_message(Topic_Name_t topic, void* data_ptr) {
    Topic_Handle_t* topic_handle = get_topic_handle(topic);
    if (topic_handle->queue_length == 1) {
        return xQueueOverwrite(topic_handle->queue_handle, data_ptr);
    } else {
        return xQueueSendToBack(topic_handle->queue_handle, data_ptr, 0);
    }
}

uint8_t pub_message_from_isr(Topic_Name_t topic, void* data_ptr,
                             uint8_t* will_context_switch) {
    BaseType_t context_switch, res;
    Topic_Handle_t* topic_handle = get_topic_handle(topic);
    if (topic_handle->queue_length == 1) {
        res = xQueueOverwriteFromISR(topic_handle->queue_handle, data_ptr,
                                     &context_switch);
    } else {
        res = xQueueSendToBackFromISR(topic_handle->queue_handle, data_ptr,
                                      &context_switch);
    }
    *will_context_switch = (uint8_t) context_switch;
    return res;
}
