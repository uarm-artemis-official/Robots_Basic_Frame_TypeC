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
    virtual bool is_valid_output(size_t motor_index, int32_t new_output) = 0;
    virtual void set_motor_voltage(uint32_t can_id, int32_t output) = 0;
    virtual void send_motor_voltage() = 0;
    virtual void request_feedback(Motor_CAN_ID_t can_id) = 0;
    virtual void get_raw_feedback(uint32_t stdid, uint8_t data[8],
                                  void* feedback) = 0;
    virtual Motor_Brand_t get_motor_brand(uint32_t stdid) = 0;
};

class IImu {
   public:
    virtual void init() = 0;
    virtual float get_temp() = 0;
    virtual void get_attitude(Attitude_t& attitude) = 0;
    virtual void get_sensor_data(AhrsSensor_t& sensor) = 0;
    virtual void set_heat_pwm(uint16_t duty_cycle) = 0;
    virtual void gather_sensor_data(AhrsSensor_t& sensor, bool read_mag) = 0;
};

// TODO: Re-enable later.
// class IRefUI {
//    public:
//     virtual void init() = 0;
//     virtual void set_ui_data(referee_ui_type_t ui_type, uint8_t robot_id,
//                              ref_ui_info_t ref_ui_info) = 0;
//     virtual void send_ui_data(uint16_t cmd_id, uint16_t len,
//                               referee_ui_type_t ui_type) = 0;
//     virtual void draw_marks() = 0;
//     virtual void draw_vaild_info(uint32_t act_mode, uint32_t level) = 0;
// };

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

class IAmmoLid {
   public:
    virtual void init() = 0;
    virtual void set_lid_status(EAmmoLidStatus new_status) = 0;
};

class IRCComm {
   public:
    virtual ~IRCComm() = default;  // Virtual destructor
    virtual void buffer_init(Buffer& buffer) = 0;
    virtual void key_object_init(KeyObject& key) = 0;
    virtual void keyboard_init(Keyboard& keyboard) = 0;
    virtual void mouse_init(Mouse& mouse) = 0;
    virtual void pc_init(PC& pc) = 0;
    virtual void controller_init(Controller& controller) = 0;
    virtual void parse_switches(Buffer& buffer, ESwitchState& s1,
                                ESwitchState& s2) = 0;
    virtual void parse_controller(Buffer& buffer, Controller& controller) = 0;
    virtual void parse_pc(Buffer& buffer, PC& pc) = 0;
    virtual void key_scan(KeyObject& key, uint16_t key_buffer,
                          EKeyBitIndex key_bit_index) = 0;
};

#endif