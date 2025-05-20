#ifndef __APP_TYPES_H
#define __APP_TYPES_H

#include "attitude_types.h"
#include "control_types.h"
#include "subsystems_interfaces.h"
#include "subsystems_types.h"
#include "uarm_types.h"

/* =========================================================================
 * PUBLIC TYPES
 * ====================================================================== */
typedef enum { CTRLER_MODE = 1, PC_MODE } CtrlMode_t;

typedef enum {
    PATROL_MODE = 1,
    DETECTION_MODE,
    AUTO_AIM_MODE,
    AUTO_PILOT_MODE,
    IDLE_MODE,
    DEBUG_MODE,
    //	PC_MODE
} BoardMode_t;  //only for sentry

typedef enum {
    GIMBAL_CENTER =
        1,  // relative angle control using encoder, chassis front always facing yaw center
    GIMBAL_FOLLOW,  // relative angle control using encoder, chassis always moving along gimbal coordinate but not align center.
    SELF_GYRO,  // relative angle control using encoder, chassis keep spinning while gimbal can move freely
    INDPET_MODE,  // chassis ground coordinate, or dummy version of self-gyro mode
} BoardActMode_t;  // should be determined by remore controller

typedef enum {
    SHOOT_ONCE = 1,
    SHOOT_TRIPLE,
    SHOOT_CONT,
    SHOOT_RESERVE,
    SHOOT_CEASE
} ShootActMode_t;

typedef enum { GYRO_MODE = 1, ENCODE_MODE } GimbalMotorMode_t;

typedef struct {
    float vx;
    float vy;
    float wz;
} Gimbal_Axis_t;  //for remote controller set gimbal dir

/* =========================================================================
 * CHASIS TYPES
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
    float vx;  //x axis velocity
    float vy;  //y axis velocity
    float wz;  //w axis angular velocity
    float max_vx;
    float max_vy;
    float max_wz;

    float gimbal_yaw_rel_angle;
    float gimbal_yaw_abs_angle;

    float gimbal_pitch_rel_angle;
    float gimbal_pitch_abs_angle;

    uint8_t pc_target_value;

    PID_t f_pid;  //for Chassis twist(in Gimbal_Center mode)
    int16_t mec_spd[4];
    int16_t swerve_spd
        [4];  // 0 Forward Left, 1 Forward Right, 2 Backward Right, 3 Backward Left (clockwise)
    float swerve_angle
        [4];  // 0 Forward Left, 1 Forward Right, 2 Backward Right, 3 Backward Left (clockwise)
    Gimbal_Axis_t gimbal_axis;

    uint16_t chassis_gear;
    uint8_t prev_robot_level;
    uint8_t cur_robot_level;
    ChassisPowerStat_t ref_power_stat;
    ChassisPowerStat_t local_power_stat;

    BoardMode_t chassis_mode;  //chassis mode selection
    BoardActMode_t chassis_act_mode;
    ChassisGearMode_t chassis_gear_mode;

} Chassis_t;

typedef struct {
    uint32_t stdid;
    PID_t f_pid;  //first pid handler for single-loop control
    Motor_Feedback_t feedback;
} Chassis_Wheel_Control_t;

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
    float pitch_imu_center;

    Gimbal_Axis_t axis;

    //	Motor_Feedback_t yaw_ecd_fb;	//yaw feedback data pool
    //	Motor_Feedback_t pitch_ecd_fb; //pitch feedback data pool

    /* algorithm related */
    ramp_t yaw_ramp;           // yaw ramp for calibration process
    ramp_t pitch_ramp;         // pitch ramp for calibration process
    AhrsSensor_t ahrs_sensor;  // copy the sensor data from imu
    Attitude_t euler_angle;    // quaternion to euler's angle

    /* filters */
    /* pc control filters */
    ewma_filter_t ewma_f_x;  //Exponential mean filtering for yaw
    ewma_filter_t ewma_f_y;  //Exponential mean filtering for pitch
    //	sliding_mean_filter_t swm_f_x; //Sliding window mean filter for yaw
    //	sliding_mean_filter_t swm_f_y; //Sliding window mean filter for pitch
    /* auto aimming */
    ewma_filter_t ewma_f_aim_yaw;
    ewma_filter_t ewma_f_aim_pitch;

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

/* =========================================================================
 * SHOOT TYPES
 * ====================================================================== */
/**
  * @brief  shoot task main struct
  */
typedef enum { OPEN = 0, CLOSE } ShootLidStatus_t;

typedef struct {
    float mag_tar_spd;
    float mag_tar_angle;  //target relative angle refered to cur_abs_position
    float mag_cur_angle;  //current actual relative angle ahs been reached
    float mag_pre_ecd_angle;
    int16_t mag_center_offset;
    int32_t mag_turns_counter;
    int16_t mag_zero_offset;
    uint8_t prev_angle_reset;

    int32_t fric_tar_spd;
    uint16_t fric_left_cur_spd;
    uint16_t fric_right_cur_spd;
    int32_t fric_can_tar_spd;
    uint8_t fric_engage_flag;
    uint32_t fric_counter;

    uint8_t lid_counter;

    Motor_Feedback_t mag_fb;
    ramp_t fric_left_ramp;
    ramp_t fric_right_ramp;
    ShootLidStatus_t lid_status;
    ShootActMode_t shoot_act_mode;
} Shoot_t;

/* =========================================================================
 * IMU TYPES
 * ====================================================================== */
typedef struct {
    PID2_t pid;
} IMU_Heat_t;

#endif