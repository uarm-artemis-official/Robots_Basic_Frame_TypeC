#ifndef __SUBSYSTEMS_CLASSES_HPP
#define __SUBSYSTEMS_CLASSES_HPP

#include "madgewick.hpp"
#include "middleware_interfaces.hpp"
#include "subsystems_interfaces.hpp"
#include "subsystems_types.hpp"
#include "uarm_types.hpp"

namespace ammo_lid {
    class AmmoLid : public IAmmoLid {
       private:
        static constexpr uint16_t CLOSED_PWM_CMP = 503;
        static constexpr uint16_t OPEN_PWM_CMP = 365;
        static constexpr MW_TIM::Timer ammo_lid_timer = MW_TIM::Timer::TIM_1;
        static constexpr MW_TIM::Channel ammo_lid_channel =
            MW_TIM::Channel::CHANNEL_1;
        LidStatus lid_status;
        MW_TIM::IPWM& pwm;

       public:
        AmmoLid(MW_TIM::IPWM& pwm_ref);

        void init() override;
        void set_lid_status(LidStatus new_status) override;
    };
}  // namespace ammo_lid

class Debug : public IDebug {
   public:
    BoardStatus_t get_board_status(void) override;
    void set_led_state(Board_LED_t led, Board_LED_State_t state) override;
};

class EventCenter : public IEventCenter {
   private:
    MW_RTOS::EventGroupHandle events_group;
    Sync_group_t sync_groups[NUM_SYNC_GROUPS];

   public:
    void init() override;
    UARM_Events_t wait_events(UARM_Events_t wait_events,
                              uint32_t timeout) override;
    void emit_events(UARM_Events_t new_events) override;
    void clear_events(UARM_Events_t clear_events) override;
    bool sync_tasks(Sync_Event_t sync_event, UARM_Events_t set_task,
                    uint32_t timeout) override;
};

class Imu : public IImu {
   private:
    Madgewick_Filter madgewick;
    float temperature;
    float gyro[3], accel[3], mag[3];

    const float accel_bias[3] = {0.076091, -0.056203, 0.049820};
    const float accel_scale[3][3] = {{1.004290, 0.002457, 0.000033},
                                     {0.002457, 1.002675, 0.002101},
                                     {0.000033, 0.002101, 0.994819}};
    const float gyro_bias[3] = {0.00127898, 0.00048873, 0.00255299};
    const float gyro_scale[3][3] = {
        {1.0, 0, 0},
        {0.0, 1.0, 0},
        {0.0, 0, 1.0},
    };
    const float orientation[3][3];

   public:
    Imu(uint32_t sampling_rate_, float beta_, const float orientation_[3][3]);
    void init() override;
    float get_temp() override;
    void get_attitude(Attitude_t& attitude) override;
    void get_sensor_data(AhrsSensor_t& sensor) override;
    void set_heat_pwm(uint16_t duty_cycle) override;
    void gather_sensor_data(AhrsSensor_t& sensor, bool read_mag) override;
    void adjust_data(float output[3], float data[3], const float bias[3],
                     const float scale[3][3]);
};

// TODO: Add timestamp or something for users to differentiate messages.
class MessageCenter : public IMessageCenter {
   private:
    // TODO: replace with std::array. Hard? cannot use template deduction??
    Topic_Handle_t topic_handles[17] = {
        Topic_Handle_t {MOTOR_SET, sizeof(MotorSetMessage_t), 5, NULL},
        Topic_Handle_t {MOTOR_READ, sizeof(MotorReadMessage_t), 1, NULL},
        Topic_Handle_t {RC_INFO, sizeof(RCInfoMessage_t), 1, NULL},
        Topic_Handle_t {COMM_OUT, sizeof(CANCommMessage_t), 5, NULL},
        Topic_Handle_t {COMM_IN, sizeof(CANCommMessage_t), 5, NULL},
        Topic_Handle_t {IMU_READINGS, sizeof(float) * 2, 1, NULL},

        // [yaw, pitch]
        Topic_Handle_t {GIMBAL_REL_ANGLES, sizeof(float) * 2, 1, NULL},

        Topic_Handle_t {PLAYER_COMMANDS, 0, 1, NULL},
        Topic_Handle_t {REFEREE_IN, sizeof(uint8_t) * 41, 1, NULL},
        Topic_Handle_t {RC_RAW, sizeof(uint8_t) * 18, 5, NULL},

        Topic_Handle_t {UC_PACK_IN, sizeof(uint8_t) * 64, 1, NULL},
        Topic_Handle_t {UC_PACK_OUT, sizeof(uint8_t) * 196, 10, NULL},
        Topic_Handle_t {AUTO_AIM, sizeof(float) * 2, 1, NULL},

        Topic_Handle_t {COMMAND_CHASSIS, sizeof(ChassisCommandMessage_t), 1,
                        NULL},
        Topic_Handle_t {COMMAND_GIMBAL, sizeof(GimbalCommandMessage_t), 1,
                        NULL},
        Topic_Handle_t {COMMAND_SHOOT, sizeof(ShootCommandMessage_t), 1, NULL},
        Topic_Handle_t {REFEREE_OUT, sizeof(RefereeInfoMessage_t), 1, NULL},
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

class Motors : public IMotors {
   private:
    uint32_t counter = 0;

   public:
    int32_t prev_swerve_data[4];
    Generic_Motor_t motors[MAX_MOTOR_COUNT];
    Motor_Config_t config;

    Motors();
    void init(Motor_Config_t config) override;
    bool is_valid_output(size_t motor_index, int32_t new_output) override;
    void set_motor_voltage(uint32_t can_id, int32_t output) override;
    void send_motor_voltage() override;
    void request_feedback(Motor_CAN_ID_t can_id) override;
    void get_raw_feedback(uint32_t stdid, uint8_t data[8],
                          void* feedback) override;
    Motor_Brand_t get_motor_brand(uint32_t stdid) override;
};

class PCComm : public IPCComm {
   public:
    uint8_t uc_check_pack_integrity(uint8_t* pack_bytes,
                                    uint8_t pack_size) override;
    uint8_t get_data_size(uint8_t header_id) override;
    void start_receive(uint8_t* pack_buffer) override;
    void restart_receive(uint8_t* pack_buffer) override;
    void send_bytes(uint8_t* bytes, uint32_t size) override;
    UC_Checksum_t calc_checksum(void* data, size_t size) override;
    uint8_t is_valid_header(uint8_t* input_buffer) override;
};

class RCComm : public IRCComm {
   public:
    void buffer_init(Buffer& buffer) override;
    void key_object_init(KeyObject& key) override;
    void keyboard_init(Keyboard& keyboard) override;
    void mouse_init(Mouse& mouse) override;
    void pc_init(PC& pc) override;
    void controller_init(Controller& controller) override;
    void parse_switches(Buffer& buffer, ESwitchState& s1,
                        ESwitchState& s2) override;
    void parse_controller(Buffer& buffer, Controller& controller) override;
    void parse_pc(Buffer& buffer, PC& pc) override;
    void key_scan(KeyObject& key, uint16_t key_buffer,
                  EKeyBitIndex key_bit_index) override;
};

class RefereeUI : public IRefUI {
   private:
    Referee_UI_t ref_ui;
    uint8_t ref_tx_frame[256];

   public:
    void init() override;
    void set_ui_data(referee_ui_type_t ui_type, uint8_t robot_id,
                     ref_ui_info_t ref_ui_info) override;
    void send_ui_data(uint16_t cmd_id, uint16_t len,
                      referee_ui_type_t ui_type) override;
    void draw_marks() override;
    void draw_vaild_info(uint32_t act_mode, uint32_t level) override;
};

#endif