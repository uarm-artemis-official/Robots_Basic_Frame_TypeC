#ifndef __MESSAGE_CENTER_H
#define __MESSAGE_CENTER_H

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

// TODO: Add timestamp or something for users to differentiate messages.
class MessageCenter : public IMessageCenter {
   private:
    // TODO: replace with std::array.
    Topic_Handle_t topic_handles[15] = {
        Topic_Handle_t {MOTOR_SET, sizeof(MotorSetMessage_t), 5, NULL},
        Topic_Handle_t {MOTOR_READ, sizeof(MotorReadMessage_t), 1, NULL},
        Topic_Handle_t {RC_INFO, sizeof(RCInfoMessage_t), 1, NULL},
        Topic_Handle_t {COMM_OUT, sizeof(CANCommMessage_t), 5, NULL},
        Topic_Handle_t {COMM_IN, sizeof(CANCommMessage_t), 5, NULL},
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
        Topic_Handle_t {UC_PACK_OUT, sizeof(uint8_t) * 196, 10, NULL},
        Topic_Handle_t {AUTO_AIM, sizeof(float) * 2, 1, NULL},
    };
    bool initialized = false;

   public:
    static MessageCenter& get_instance();

    void init() override;

    // TODO: Replace return values with bools?
    uint8_t get_message(Topic_Name_t topic, void* data_ptr,
                        int ticks_to_wait) override;
    uint8_t peek_message(Topic_Name_t topic, void* data_ptr,
                         int ticks_to_wait) override;
    uint8_t pub_message(Topic_Name_t topic, void* data_ptr) override;
    uint8_t pub_message_from_isr(Topic_Name_t topic, void* data_ptr,
                                 uint8_t* will_context_switch) override;
    Topic_Handle_t& get_topic_handle(Topic_Name_t name) override;
};

#endif
