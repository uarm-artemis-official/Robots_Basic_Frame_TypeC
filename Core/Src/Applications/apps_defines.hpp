#ifndef __APPS_CONFIG_H
#define __APPS_CONFIG_H

#include <cstdint>

/* =========================================================================
 * PUBLIC DEFINES 
 * ====================================================================== */
/* Task exec time in milliseconds */
namespace app_defines {
    constexpr uint32_t chassis_task_loop_period_ms = 5;
    constexpr uint32_t gimbal_task_loop_period_ms = 5;
    constexpr uint32_t shoot_task_loop_period_ms = 5;
    constexpr uint32_t imu_task_loop_period_ms = 2;
    constexpr uint32_t timer_task_loop_period_ms = 1;
    constexpr uint32_t comm_task_loop_period_ms = 2;
    constexpr uint32_t rc_task_loop_period_ms = 2;
    constexpr uint32_t pc_uart_task_loop_period_ms = 1;
    constexpr uint32_t referee_task_loop_period_ms = 10;
}  // namespace app_defines

/* =========================================================================
 * CHASSIS DEFINES 
 * ====================================================================== */
/* define general declarations for gimbal task here */
// TODO: Use strong types.
#define CHASSIS_WHEEL_X_LENGTH (0.40f)      // meters
#define CHASSIS_WHEEL_Y_LENGTH (0.35f)      // meters
#define CHASSIS_OMNI_WHEEL_RADIUS (0.076f)  // meters

// TODO: Correct reduction ratio to proper one for M3508
#define CHASSIS_MOTOR_DEC_RATIO (19.0f)  // motor deduction ratio 19:1
#define CHASSIS_SLEF_GYRO_ANG_VEL 80
#define CHASSIS_MAX_SPEED 16384

// Wheel index defines
#define CHASSIS_WHEEL1_INDEX 0
#define CHASSIS_WHEEL2_INDEX 1
#define CHASSIS_WHEEL3_INDEX 2
#define CHASSIS_WHEEL4_INDEX 3

#define CHASSIS_MAX_WHEELS 4

/* =========================================================================
 * GIMBAL DEFINES
 * ====================================================================== */
#define PITCH_ECD_DELTA 1364  //60/180*4096
#define PITCH_GEAR_RATIO 1    // The ratio of the gear box of the pitch motor
#define PITCH_GYRO_DELTA (20.0f * DEGREE2RAD * PITCH_GEAR_RATIO)

#define YAW_GEAR_RATIO 1.0f  //if install a gear, calc the gear ratio here

#define GIMBAL_YAW_MOTOR_INDEX 0
#define GIMBAL_PITCH_MOTOR_INDEX 1
#define GIMBAL_MOTOR_COUNT 2

#define GIMBAL_IMU_SAMPLES 20

/* =========================================================================
 * IMU DEFINES
 * ====================================================================== */
#define IMU_TMP_PWM_HTIM htim10
#define IMU_TMP_PWM_CHANNEL TIM_CHANNEL_1

/* =========================================================================
 * SHOOT DEFINES
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define USE_CAN_FRIC 1  //if use 3508 instead of pwm-based fric wheel motor

//Max input value (abs) for magazine motor, for p2006, it is 10000, f
//or 3508 it is 16000, for 6020 it is 30000
#define MAX_PWM_ON_TIME 2000
#define MIN_PWM_ON_TIME 1000

/* 2305 can value*/
#define LEVEL_ONE_PWM 300

#define FRIC_PWM_DELAY 10
#define FRIC_CAN_RAMP_DELAY 40

/* 3508 can value*/
//#define LEVEL_ONE_CAN_SPD 7500 // 28: 7500, 27:7350
#define LEVEL_ONE_CAN_SPD 7500

#define SHOOT_ONCE_MAG_ANGLE (20.0f * DEGREE2RAD)
#define SHOOT_CONT_MAG_SPEED 1.0 * PI  //rpm/sec
#define SHOOT_MAG_GEAR_RATIO 19
#define SHOOT_REVERSE_MAG_SPEED 0.2 * PI

#define SHOOT_LEFT_FRIC_WHEEL_INDEX 0
#define SHOOT_RIGHT_FRIC_WHEEL_INDEX 1

/* =========================================================================
 * COMM DEFINES
 * ====================================================================== */
/* Define used can id */
#define CHANNEL_OFFSET_MAX_ABS_VAL 660
#define MAX_MOUSE_YAW_OUT 2
#define MAX_MOUSE_PITCH_OUT 2

/* =========================================================================
 * TIMER DEFINES
 * ====================================================================== */
// #define DISABLE_MOTOR_SEND

/* =========================================================================
 * TIMER DEFINES
 * ====================================================================== */
#define RC_YAW_SEND_THRESHOLD 0.001f
#define RC_PITCH_SEND_THRESHOLD 0.001f
#define RC_WZ_SEND_THRESHOLD 0.01f
#define RC_V_PERP_SEND_THRESHOLD 0.01f
#define RC_V_PARALLEL_SEND_THRESHOLD 0.01f

/* =========================================================================
 * REFEREE DEFINES
 * ====================================================================== */
#define REFEREE_NON_RECV_MAX_COUNT \
    100  // maximum count of non-received referee data before reset
// now is 1 second

#endif