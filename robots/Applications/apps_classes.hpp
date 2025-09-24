#ifndef __APPS_CLASSES_HPP
#define __APPS_CLASSES_HPP

#include "apps_interfaces.hpp"
#include "apps_types.hpp"
#include "subsystems_interfaces.hpp"
#include "subsystems_modules.hpp"

template <class DriveTrain>
class ChassisApp : public RTOSApp<ChassisApp<DriveTrain>,
                                  apps_defines::chassis_task_loop_period_ms> {
   private:
    ChassisDrive<DriveTrain>& drive_train;
    mc2::RobotMC& mc;
    IDebug& debug;

    Chassis_t chassis;

   public:
    static constexpr float MAX_TRANSLATION = 2;  // in m/s
    static constexpr float MAX_ROTATION = PI;    // rad/s
    static constexpr float GYRO_SPEED = PI;

    explicit ChassisApp(MW_RTOS::IRTOS& _rtos, DriveTrain& drive_train_ref,
                        mc2::RobotMC& mc_ref, IDebug& debug_ref);
    void init();
    void set_initial_state();

    void loop();

    void calc_movement_vectors();

    void process_commands();
    void chassis_get_gimbal_rel_angles();

    void set_board_mode(BoardMode_t new_board_mode);
    void set_act_mode(BoardActMode_t new_act_mode);
};

// TODO: Move DriveTrains into policies or some other kind of subdirectory.
class OmniDrive : public ChassisDrive<OmniDrive> {
   private:
    std::array<Chassis_Wheel_Control_t, 4> motor_controls;
    std::array<float, 4> motor_angular_vel;
    std::array<float, 4> wheel_power_consumption;
    mc2::RobotMC& mc;
    IMotors& motors;
    float width, length, power_limit, chassis_dt;
    const float a = 0;
    const float k1 = 0;
    const float k2 = 0;

   public:
    explicit OmniDrive(mc2::RobotMC& mc2_ref, IMotors& motors,
                       float chassis_width, float chassis_length,
                       float power_limit_, float chassis_dt_);

    void init_impl();

    void get_motor_feedback();

    void calc_target_motor_speeds(float vx, float vy, float wz);
    void calc_motor_volts();
    void calc_motor_outputs(float vx, float vy, float wz);
    void calc_power_limits();

    void calc_wheel_power_consumption();
    float calc_power_consumption();
    void set_max_power_impl(float new_max_power);

    void send_motor_messages();
};

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    static constexpr size_t NUM_STEER_MOTORS = 4;
    static constexpr size_t NUM_DRIVE_MOTORS = 4;

    mc2::RobotMC& mc;
    IMotors& motors;
    const float width;
    const float dt;

    std::array<Swerve_Drive_Control_t, NUM_DRIVE_MOTORS> drive_motors;
    std::array<Swerve_Steer_Control_t, NUM_STEER_MOTORS> steer_motors;

    std::array<float, NUM_STEER_MOTORS> steer_curr_angle;
    std::array<int16_t, NUM_STEER_MOTORS> steer_curr_speed;
    std::array<float, NUM_STEER_MOTORS> steer_cw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_ccw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_target_angle;
    std::array<float, NUM_DRIVE_MOTORS> drive_target_speed;

    std::array<uint16_t, NUM_STEER_MOTORS> steer_max_speed;
    std::array<bool, NUM_STEER_MOTORS> steer_ccw;
    std::array<float, NUM_STEER_MOTORS> steer_output_angle;
    std::array<int32_t, NUM_DRIVE_MOTORS> drive_output;

   public:
    static int32_t pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                         uint32_t angle);

    explicit SwerveDrive(mc2::RobotMC& mc2_ref, IMotors& motors_ref,
                         float width_, float dt_);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

/*
 * @attention:
 * 		We transit each angle mentioned here to radian because:
 * 		1. Unit and normalize the calculation of PID (spec for angular loop)
 * 		2. for safety consideration, sometimes swap the mode between gyro and ecd
 * 		   keep radians helping unit the input value
 * 		3. easier for calculating cos/sin functions
 *
 * */
class GimbalApp
    : public ExtendedRTOSApp<GimbalApp,
                             apps_defines::gimbal_task_loop_period_ms> {
   private:
    Gimbal_t gimbal;
    Gimbal_Imu_Calibration_t imu_calibration;
    Gimbal_Motor_Control_t motor_controls[GIMBAL_MOTOR_COUNT];
    int16_t gimbal_channels[2];
    float command_deltas[2];

    mc2::RobotMC& mc;
    IEventCenter& event_center;
    IDebug& debug;
    IMotors& motors;

   public:
    // Software limits on pitch targets to prevent pitch from hitting mechanical hard stops.
    static constexpr float PITCH_LOWER_LIMIT = -0.1;
    static constexpr float PITCH_UPPER_LIMIT = 0.4;
    static constexpr uint32_t IMU_CENTER_TARGET_SAMPLES = 100;

    static float calc_rel_angle(float angle1, float angle2);
    static int16_t calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);

    explicit GimbalApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                       IEventCenter& event_center, IDebug& debug_ref,
                       IMotors& motors_ref);
    void init();
    void set_initial_state();
    bool calibrate_start_precondition();
    void wait_for_motors();

    bool exit_calibrate_cond();
    void calibrate();

    void loop();

    bool is_imu_calibrated();

    void set_modes(uint8_t modes[3]);
    void set_board_mode(BoardMode_t mode);
    void set_act_mode(BoardActMode_t mode);
    void set_motor_mode(GimbalMotorMode_t mode);
    void safe_mode_switch();

    void get_motor_feedback();
    void get_imu_headings();

    void process_commands();

    void calc_imu_center();

    void update_imu_angle(float yaw, float pitch);
    void update_ecd_angles();
    void update_headings();
    void update_targets();

    void limit_pitch_target();

    void cmd_exec();
    void calc_control_signals();

    void send_motor_volts();
    void send_rel_angles();
};

class ShootApp
    : public RTOSApp<ShootApp, apps_defines::shoot_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    IAmmoLid& ammo_lid;
    IMotors& motors;

    Shoot shoot;
    LoaderControl loader_control;
    FlyWheelControl flywheel_controls[2];
    const float LOADER_ACTIVE_RPM;
    const float FLYWHEEL_ACTIVE_TARGET_RPM;
    const float MAX_FLYWHEEL_ACCEL;

   public:
    explicit ShootApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                      IAmmoLid& ammo_lid_ref, IMotors& motors_ref,
                      float loader_active_rpm_, float flywheel_target_rpm_,
                      float max_flywheel_accel);

    void init();
    void loop();

    void get_motor_feedback();
    void detect_loader_stall();

    void process_commands();

    void calc_targets();
    void calc_motor_outputs();
    void send_motor_outputs();

    void set_shoot_mode(ShootActMode_t new_mode);
    void set_loader_target(float new_target);
    void set_flywheel_target(float new_target);
};

class IMUApp
    : public ExtendedRTOSApp<IMUApp, apps_defines::imu_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    IEventCenter& event_center;
    IImu& imu;
    IDebug& debug;

    IMU_t imu_app_state;
    IMU_Heat_t imu_heating_control;
    Attitude_t attitude;
    AhrsSensor_t sensor_data;

   public:
    static constexpr float TARGET_IMU_TEMP = 40.0f;
    static constexpr float NORMAL_TEMP_THRESHOLD = 1.0f;
    static constexpr float IMU_RESET_THRESHOLD = 7.0f;

    explicit IMUApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                    IEventCenter& event_center_ref, IImu& imu_ref,
                    IDebug& debug_ref);
    void init();
    void calibrate();
    bool exit_calibrate_cond();
    void loop();

    int32_t imu_temp_pid_control();

    void set_imu_temp_status(IMU_temp_status status);
};

class RefereeApp
    : public RTOSApp<RefereeApp, apps_defines::referee_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    IEventCenter& event_center;
    IDebug& debug;
    IRefUI& ref_ui;  // Referee UI interface

    Referee_t ref;
    uint16_t non_recv_count =
        0;  // Count the number of times referee data is not received
    uint8_t ui_sendig_count = 0;  // Count the number of times UI data is sent

   public:
    explicit RefereeApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                        IEventCenter& evt_center, IDebug& debug,
                        IRefUI& ref_ui);
    void init();
    void loop();

    void read_ref_data();
    void draw_all_ui();
    void reset();
};

class RCApp : public RTOSApp<RCApp, apps_defines::rc_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    IRCComm& rc_comm;

    // TODO: remove and replace with rc_rx_buffer.
    mc2::RCRaw rc_raw;
    Buffer rc_rx_buffer;
    RemoteControl_t rc;
    uint32_t rc_idle_count = 0;

    BoardMode_t pc_board_mode;
    BoardActMode_t pc_act_mode;
    ShootActMode_t pc_shoot_mode;
    ammo_lid::LidStatus pc_ammo_status;

   public:
    explicit RCApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                   IRCComm& rc_comm_ref);

    void init();
    void loop();

    void parse_raw_rc();
    void map_switches_to_modes(BoardMode_t& board_mode,
                               BoardActMode_t& act_mode,
                               ShootActMode_t& shoot_mode);
    void detect_rc_loss();
    void send_gimbal_can_comm(float yaw, float pitch, BoardMode_t board_mode,
                              BoardActMode_t act_mode);
    void send_chassis_command(float v_parallel, float v_perp, float wz,
                              BoardMode_t board_mode, BoardActMode_t act_mode);
    void send_shoot_command(ShootActMode_t shoot_mode,
                            ammo_lid::LidStatus ammo_lid_status);

    void pub_command_messages();
};

class TimerApp
    : public RTOSApp<TimerApp, apps_defines::timer_task_loop_period_ms> {
   private:
    const std::array<Motor_CAN_ID_t, 4> swerve_ids = {
        SWERVE_STEER_MOTOR1, SWERVE_STEER_MOTOR2, SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4};
    IMotors& system_motors;
    mc2::RobotMC& mc;
    IDebug& debug;
    mc2::MotorSet motor_set;
    MotorSetMessage_t motor_tx_message;
    BoardStatus_t board_status;

   public:
    explicit TimerApp(MW_RTOS::IRTOS& _rtos, IMotors& system_motors_ref,
                      mc2::RobotMC& mc2_ref, IDebug& debug_ref);
    void init();
    void loop();
};

class PCUARTApp
    : public RTOSApp<PCUARTApp, apps_defines::pc_uart_task_loop_period_ms> {
   private:
    mc2::RobotMC& mc;
    IMotors& motors;
    IPCComm& pc_comm;

    uint32_t idle_count = 0;
    mc2::UCPackIn uc_pack_in;
    uint8_t new_send_buffer[196];
    float recent_deltas[2];

   public:
    explicit PCUARTApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                       IMotors& motors_, IPCComm& pc_comm_);
    void init();
    void loop();
    void send_swerve_data();
};

namespace CommApp {
    class CommApp
        : public RTOSApp<CommApp, apps_defines::comm_task_loop_period_ms> {
       private:
        mc2::RobotMC& mc;
        IDebug& debug;
        MW_CAN::ICAN& can;
        BoardStatus_t board_status;
        Config config;

       public:
        explicit CommApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc2_ref,
                         IDebug& debug, MW_CAN::ICAN& can, Config config);
        void init();
        void loop();
        bool transmit_interboard_message(const uint32_t message_id,
                                         const uint8_t message_data[8]);
    };
}  // namespace CommApp

#endif