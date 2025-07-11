#ifndef __APPS_CONFIG_H
#define __APPS_CONFIG_H

/* =========================================================================
 * PUBLIC DEFINES 
 * ====================================================================== */
/* Task exec time in milliseconds */
#define CHASSIS_TASK_EXEC_TIME 5
#define GIMBAL_TASK_EXEC_TIME 5
#define SHOOT_TASK_EXEC_TIME 5
#define IMU_TASK_EXEC_TIME 2
#define TIMER_TASK_EXEC_TIME 1
#define COMM_TASK_EXEC_TIME 2
#define RC_TASK_EXEC_TIME 2
#define PC_UART_TASK_EXEC_TIME 1
#define REFEREE_TASK_EXEC_TIME 10

/* =========================================================================
 * CHASSIS DEFINES 
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define CHASSIS_WHEEL_X_LENGTH (0.40f)      // meters
#define CHASSIS_WHEEL_Y_LENGTH (0.35f)      // meters
#define CHASSIS_OMNI_WHEEL_RADIUS (0.076f)  // meters

// TODO: Correct reduction ratio to proper one for M3508
#define CHASSIS_MOTOR_DEC_RATIO (19.0f)  // motor deduction ratio 19:1
#define CHASSIS_SLEF_GYRO_ANG_VEL 80
#define CHASSIS_MAX_SPEED 16384

/* power limit defines */
#define CHASSIS_POWER_DANGER 20       //random value, test soon
#define CHASSIS_POWER_THRESHOLD 45    //random value, test soon
#define POWER_TO_CURRENT (1.0f)       //random value, test soon
#define CHASSIS_PC_RAMP_VALUE (0.5f)  //ramp value for increment of the motors

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

#endif

/* =========================================================================
 * TIMER DEFINES
 * ====================================================================== */
// #define DISABLE_MOTOR_SEND

/* Referee App Defines */
