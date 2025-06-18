
// TODO: Rename to apps_types.hpp
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

/* =========================================================================
 * OMNI DRIVE TYPES
 * ====================================================================== */
typedef struct {
    uint32_t stdid;
    PID2_t f_pid;  //first pid handler for single-loop control
    Motor_Feedback_t feedback;
} Chassis_Wheel_Control_t;

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

/* =========================================================================
 * IMU TYPES
 * ====================================================================== */
typedef struct {
    Prescaled_PID2_t pid;
} IMU_Heat_t;

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
#endif