#ifndef __SUBSYSTEMS_INTERFACES_H
#define __SUBSYSTEMS_INTERFACES_H

#include "subsystems_types.h"

class IMessageCenter {
   public:
    virtual void init() = 0;
    virtual uint8_t get_message(Topic_Name_t topic, void* data_ptr,
                                int ticks_to_wait) = 0;
    virtual uint8_t peek_message(Topic_Name_t topic, void* data_ptr,
                                 int ticks_to_wait) = 0;
    virtual uint8_t pub_message(Topic_Name_t topic, void* data_ptr) = 0;
    virtual uint8_t pub_message_from_isr(Topic_Name_t topic, void* data_ptr,
                                         uint8_t* will_context_switch) = 0;
    virtual Topic_Handle_t& get_topic_handle(Topic_Name_t name) = 0;
};

class IMotors {
   public:
    virtual void init(Motor_Config_t config) = 0;
    virtual void set_motor_voltage(Motor_CAN_ID_t can_id, int32_t output) = 0;
    virtual void send_motor_voltage() = 0;
};

class IImu {
   public:
    virtual void init() = 0;
    virtual float get_temp() = 0;
    virtual void get_attitude(Attitude_t* attitude) = 0;
    virtual void set_offset() = 0;
    virtual void set_heat_pwm(uint16_t duty_cycle) = 0;
    virtual void set_cali_slove() = 0;
    virtual void ahrs_update(AhrsSensor_t* sensor, bool read_mag) = 0;
};

class IEventCenter {
   public:
    virtual void init() = 0;
    virtual UARM_Events_t wait_events(UARM_Events_t wait_events,
                                      uint32_t timeout) = 0;
    virtual void emit_events(UARM_Events_t new_events) = 0;
    virtual void clear_events(UARM_Events_t clear_events) = 0;
    virtual bool sync_tasks(Sync_Event_t sync_event, UARM_Events_t set_task,
                            uint32_t timeout) = 0;
};

class IDebug {
   public:
    virtual BoardStatus_t get_board_status(void) = 0;
    virtual void set_led_state(Board_LED_t led, Board_LED_State_t state) = 0;
};

class ICanComm {
   public:
    virtual void init() = 0;
    virtual void can_transmit_comm_message(uint8_t send_data[8],
                                           uint32_t comm_id) = 0;
};

#endif