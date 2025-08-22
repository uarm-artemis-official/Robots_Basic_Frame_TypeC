
// TODO: Rename to apps_types.hpp
// TODO: Add namespaces?
// TODO: Add injectable configuration structs apps.
#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include "apps_defines.hpp"
#include "apps_interfaces.hpp"
#include "attitude_types.h"
#include "control_types.hpp"
#include "subsystems_interfaces.h"
#include "subsystems_types.hpp"
#include "uarm_types.hpp"

/* =========================================================================
 * PUBLIC TYPES
 * ====================================================================== */
typedef enum { CTRLER_MODE = 1, PC_MODE } CtrlMode_t;

typedef enum {
    PATROL_MODE = 1,
    AUTO_AIM_MODE,
    AUTO_PILOT_MODE,  // full control to mini-pc.
    IDLE_MODE,
    //	PC_MODE
} BoardMode_t;

typedef enum {
    GIMBAL_CENTER =
        1,  // relative angle control using encoder, chassis front always facing yaw center
    GIMBAL_FOLLOW,  // relative angle control using encoder, chassis always moving along gimbal coordinate but not align center.
    SELF_GYRO,  // relative angle control using encoder, chassis keep spinning while gimbal can move freely
    INDPET_MODE,  // chassis ground coordinate, or dummy version of self-gyro mode
} BoardActMode_t;  // should be determined by remore controller

typedef enum { SHOOT_CONT = 1, SHOOT_CEASE } ShootActMode_t;
enum class ShootState {
    NORMAL,
    ANTIJAM,
};

typedef enum { GYRO_MODE = 1, ENCODE_MODE } GimbalMotorMode_t;

typedef struct {
    float vx;
    float vy;
    float wz;
} Gimbal_Axis_t;  //for remote controller set gimbal dir

/* =========================================================================
 * CHASSIS TYPES
 * ====================================================================== */
typedef enum {
    AUTO_GEAR = 0,  //referee system up, auto-adjust chassis spd limit
    MANUAL_GEAR     //referee system down, manual-adjust chassis spd limit
} ChassisGearMode_t;

typedef enum {
    NO_GEAR = 0,
    GEAR_LOW = 500,
    GEAR_MID = 1000,
    GEAR_HIGH = 2000
} ChassisGearValue_t;

typedef struct {
    uint16_t current;
    float power;
    uint16_t buffer_energy;
} ChassisPowerStat_t;

typedef struct {
    /**
     * Inverse kinematic outputs.
     * 2 velocities (m/s) and 1 angular velocity (rad/s).
     * x-y-z axes follows right-hand rule with right-front-up (robot reference frame).
     * Origin is ideally at robot's center of mass.
     * i.e. positive x-axis = robot's right, and positive y-axis = robot's front.
     */
    float vx;
    float vy;
    float wz;
    float max_vx;
    float max_vy;
    float max_wz;

    /**
     * Translation variables used for calculating Inverse Kinematics.
     * These are velocity components pointing parallel and perpendicular to the movement
     * vector created by the gimbal. These are directly manipulated through controller
     * inputs and transformed into vx, and vy for chassis movement.
     */
    float v_parallel;
    float v_perp;

    /** 
     * Gimbal yaw in radians [-pi, pi] received over CAN2 from gimbal board to chassis board.
     * This is used calculating chassis inverse kinematics for moving relative to gimbal.
     * Yaw angle is CCW positive and relative to front of the robot.
     * e.g. 0 = front, -pi/2 = right, and pi/2 = left
    */
    float gimbal_yaw_rel_angle;

    PID2_t spin_pid;  //for Chassis twist(in Gimbal_Center mode)

    uint16_t chassis_gear;
    ChassisPowerStat_t ref_power_stat;
    ChassisPowerStat_t local_power_stat;

    BoardMode_t chassis_mode;  //chassis mode selection
    BoardActMode_t chassis_act_mode;
    ChassisGearMode_t chassis_gear_mode;
} Chassis_t;

template <class DriveTrain>
class ChassisApp : public RTOSApp<ChassisApp<DriveTrain>,
                                  app_defines::chassis_task_loop_period_ms> {
   private:
    ChassisDrive<DriveTrain>& drive_train;
    IMessageCenter& message_center;
    IDebug& debug;

    Chassis_t chassis;

   public:
    static constexpr float MAX_TRANSLATION = 2;  // in m/s
    static constexpr float MAX_ROTATION = PI;    // rad/s
    static constexpr float GYRO_SPEED = PI;

    ChassisApp(DriveTrain& drive_train_ref, IMessageCenter& message_center_ref,
               IDebug& debug_ref);
    void init();
    void set_initial_state();

    void loop();

    void calc_movement_vectors();

    void process_commands();
    void chassis_get_gimbal_rel_angles();

    void set_board_mode(BoardMode_t new_board_mode);
    void set_act_mode(BoardActMode_t new_act_mode);
};

/* =========================================================================
 * OMNI DRIVE TYPES
 * ====================================================================== */
typedef struct {
    uint32_t stdid;
    PID2_t f_pid;  //first pid handler for single-loop control
    Ramp sp_ramp;
    Motor_Feedback_t feedback;
} Chassis_Wheel_Control_t;

class OmniDrive : public ChassisDrive<OmniDrive> {
   private:
    std::array<Chassis_Wheel_Control_t, 4> motor_controls;
    std::array<float, 4> motor_angular_vel;
    std::array<float, 4> wheel_power_consumption;
    IMessageCenter& message_center;
    IMotors& motors;
    float width, length, power_limit, chassis_dt;
    const float a = 0;
    const float k1 = 0;
    const float k2 = 0;

   public:
    OmniDrive(IMessageCenter& message_center_ref, IMotors& motors,
              float chassis_width, float chassis_length, float power_limit_,
              float chassis_dt_);

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

/* =========================================================================
 * SWERVE DRIVE TYPES
 * ====================================================================== */
typedef struct {
    uint32_t stdid;
    PID2_t f_pid;  //first pid handler for single-loop control
    Motor_Feedback_t feedback;
    LK_Motor_Torque_Feedback_t lk_feedback;
    uint32_t angle;
} Swerve_Wheel_Control_t;

typedef struct {
    uint32_t stdid;
    PID2_t f_pid;
    Motor_Feedback_t feedback;
} Swerve_Drive_Control_t;

typedef struct {
    uint32_t stdid;
    LK_Motor_Torque_Feedback_t lk_feedback;
} Swerve_Steer_Control_t;

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    static constexpr size_t NUM_STEER_MOTORS = 4;
    static constexpr size_t NUM_DRIVE_MOTORS = 4;

    IMessageCenter& message_center;
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

    explicit SwerveDrive(IMessageCenter& message_center_ref,
                         IMotors& motors_ref, float width_, float dt_);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

/* =========================================================================
 * GIMBAL TYPES
 * ====================================================================== */

typedef struct Gimbal_t {
    /* gimbal position related */
    float yaw_target_angle;
    float yaw_rel_angle;
    float yaw_ecd_angle;
    float yaw_imu_angle;

    float pitch_target_angle;
    float pitch_rel_angle;
    float pitch_ecd_angle;
    float pitch_imu_angle;

    int16_t yaw_ecd_center;    //center position of the yaw motor by encoder
    int16_t pitch_ecd_center;  //center position of the pitch motor by encoder

    float yaw_imu_center;
    float yaw_imu_center_cumsum;
    uint32_t yaw_imu_center_sample_count;

    first_order_low_pass_t
        folp_f_yaw;  //first order low pass filter for imu data
    first_order_low_pass_t
        folp_f_pitch;  //first order low pass filter for imu data;

    GimbalMotorMode_t gimbal_motor_mode;       //gyro or encoder
    GimbalMotorMode_t prev_gimbal_motor_mode;  //gyro or encoder
    BoardActMode_t gimbal_act_mode;  //gimbal center, gimbal follow, etc
    BoardActMode_t prev_gimbal_act_mode;
    BoardMode_t gimbal_mode;  //idle(safe) or normal
} Gimbal_t;

typedef struct {
    uint32_t stdid;
    PID2_t f_pid;
    PID2_t s_pid;
    Motor_Feedback_t feedback;
} Gimbal_Motor_Control_t;

typedef struct {
    uint8_t sample_count;
    float yaw_samples_cumsum;
    float pitch_samples_cumsum;
} Gimbal_Imu_Calibration_t;

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
                             app_defines::gimbal_task_loop_period_ms> {
   private:
    Gimbal_t gimbal;
    Gimbal_Imu_Calibration_t imu_calibration;
    Gimbal_Motor_Control_t motor_controls[GIMBAL_MOTOR_COUNT];
    int16_t gimbal_channels[2];
    float command_deltas[2];

    IMessageCenter& message_center;
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

    GimbalApp(IMessageCenter& message_center_ref, IEventCenter& event_center,
              IDebug& debug_ref, IMotors& motors_ref);
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

/* =========================================================================
 * SHOOT TYPES
 * ====================================================================== */
/**
  * @brief  shoot task main struct
  */

struct Shoot {
    float loader_target_rpm;
    float flywheel_target_rpm;

    EAmmoLidStatus lid_status;
    ShootActMode_t shoot_act_mode;
    ShootState shoot_state;

    float stall_duration;
    float no_stall_duration;
    float antijam_direction;

    uint32_t loader_delay_counter;
};

struct LoaderControl {
    Motor_CAN_ID_t stdid;
    PID2_t speed_pid;
    Motor_Feedback_t feedback;
};

struct FlyWheelControl {
    Motor_CAN_ID_t stdid;
    PID2_t speed_pid;
    Motor_Feedback_t feedback;
    Ramp sp_ramp;
};

class ShootApp
    : public RTOSApp<ShootApp, app_defines::shoot_task_loop_period_ms> {
   private:
    IMessageCenter& message_center;
    IAmmoLid& ammo_lid;
    IMotors& motors;

    Shoot shoot;
    LoaderControl loader_control;
    FlyWheelControl flywheel_controls[2];
    const float LOADER_ACTIVE_RPM;
    const float FLYWHEEL_ACTIVE_TARGET_RPM;
    const float MAX_FLYWHEEL_ACCEL;

   public:
    ShootApp(IMessageCenter& message_center_ref, IAmmoLid& ammo_lid_ref,
             IMotors& motors_ref, float loader_active_rpm_,
             float flywheel_target_rpm_, float max_flywheel_accel);

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

/* =========================================================================
 * IMU TYPES
 * ====================================================================== */
typedef struct {
    Prescaled_PID2_t pid;
} IMU_Heat_t;

class IMUApp
    : public ExtendedRTOSApp<IMUApp, app_defines::imu_task_loop_period_ms> {
   private:
    IMessageCenter& message_center;
    IEventCenter& event_center;
    IImu& imu;
    IDebug& debug;

    IMU_t imu_app_state;
    IMU_Heat_t imu_heating_control;
    Attitude_t attitude;
    AhrsSensor_t sensor_data;
    float message_data[2];

   public:
    static constexpr float TARGET_IMU_TEMP = 40.0f;
    static constexpr float NORMAL_TEMP_THRESHOLD = 1.0f;
    static constexpr float IMU_RESET_THRESHOLD = 7.0f;

    IMUApp(IMessageCenter& message_center_ref, IEventCenter& event_center_ref,
           IImu& imu_ref, IDebug& debug_ref);
    void init();
    void calibrate();
    bool exit_calibrate_cond();
    void loop();

    int32_t imu_temp_pid_control();

    void set_imu_temp_status(IMU_temp_status status);
};

/* =========================================================================
 * REFEREE SYSTEM TYPES
 * ====================================================================== */
typedef struct {
    frame_header_t header;
    referee_id_t id;
    game_status_t game_status_data;
    game_result_t game_result_data;
    game_robot_HP_t robot_HP_data;
    robot_status_t robot_status_data;
    power_heat_data_t power_heat_data;
    shoot_data_t shoot_data;
    ref_ui_info_t ref_info_data;

    uint8_t ref_data[MAX_REF_RX_DATA_LEN];  // MAX_REF_RX_DATA_LEN
    // rx data
    uint8_t ref_rx_frame[MAX_REF_BUFFER_SIZE];  // MAX_REF_BUFFER_SIZE
    uint16_t ref_cmd_id;
    robot_color_t robot_color;

} Referee_t;

class RefereeApp
    : public RTOSApp<RefereeApp, app_defines::referee_task_loop_period_ms> {
   private:
    IMessageCenter& message_center;
    IEventCenter& event_center;
    IDebug& debug;
    IRefUI& ref_ui;  // Referee UI interface

    Referee_t ref;
    uint16_t non_recv_count =
        0;  // Count the number of times referee data is not received
    uint8_t ui_sendig_count = 0;  // Count the number of times UI data is sent

   public:
    RefereeApp(IMessageCenter& msg_center, IEventCenter& evt_center,
               IDebug& debug, IRefUI& ref_ui);
    void init();
    void loop();

    void read_ref_data();
    void draw_all_ui();
    void reset();
};

/* =========================================================================
 * RC TYPES
 * ====================================================================== */
typedef struct {
    /* controll mode selection */
    Controller ctrl;
    PC pc;
    CtrlMode_t control_mode;

    /* status update */
    BoardMode_t board_mode;
    BoardActMode_t board_act_mode;
} RemoteControl_t;

class RCApp : public RTOSApp<RCApp, app_defines::rc_task_loop_period_ms> {
   private:
    IMessageCenter& message_center;
    IRCComm& rc_comm;

    // TODO: remove and replace with rc_rx_buffer.
    uint8_t tmp_rx_buffer[18];
    Buffer rc_rx_buffer;
    RemoteControl_t rc;
    uint32_t rc_idle_count = 0;

    BoardMode_t pc_board_mode;
    BoardActMode_t pc_act_mode;
    ShootActMode_t pc_shoot_mode;
    EAmmoLidStatus pc_ammo_status;

   public:
    explicit RCApp(IMessageCenter& message_center_ref, IRCComm& rc_comm_ref);

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
                            EAmmoLidStatus ammo_lid_status);

    void pub_command_messages();
};

/* =========================================================================
 * COMM TYPES
 * ====================================================================== */
namespace CommApp {
    enum class OperationMode { Normal, Loopback };

    struct Config {
        OperationMode op_mode;
    };

    class CommApp
        : public RTOSApp<CommApp, app_defines::comm_task_loop_period_ms> {
       private:
        IMessageCenter& message_center;
        IDebug& debug;
        ICanComm& can_comm;
        BoardStatus_t board_status;
        Config config;

       public:
        CommApp(IMessageCenter& message_center, IDebug& debug,
                ICanComm& can_comm, Config config);
        void loop();
        void init();
    };
}  // namespace CommApp

/* =========================================================================
 * TIMER TYPES
 * ====================================================================== */
class TimerApp
    : public RTOSApp<TimerApp, app_defines::timer_task_loop_period_ms> {
   private:
    const std::array<Motor_CAN_ID_t, 4> swerve_ids = {
        SWERVE_STEER_MOTOR1, SWERVE_STEER_MOTOR2, SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4};
    IMotors& system_motors;
    IMessageCenter& message_center;
    IDebug& debug;
    MotorSetMessage_t motor_tx_message;
    BoardStatus_t board_status;

   public:
    TimerApp(IMotors& system_motors_ref, IMessageCenter& message_center_ref,
             IDebug& debug_ref);
    void init();
    void loop();
};

/* =========================================================================
 * PC UART TYPES
 * ====================================================================== */
class PCUARTApp
    : public RTOSApp<PCUARTApp, app_defines::pc_uart_task_loop_period_ms> {
   private:
    IMessageCenter& message_center;
    IMotors& motors;
    IPCComm& pc_comm;

    uint32_t idle_count = 0;
    uint8_t new_pack_buffer[64];  // TODO: Make same as MAX_PACK_BUFFER_SIZE.
    uint8_t new_send_buffer[196];
    float recent_deltas[2];

   public:
    PCUARTApp(IMessageCenter& message_center_ref, IMotors& motors_,
              IPCComm& pc_comm_);
    void init();
    void loop();
    void send_swerve_data();
};

#endif