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
#define PC_UART_TASK_EXEC_TIME 10
#define REFEREE_TASK_EXEC_TIME 10

/* =========================================================================
 * CHASSIS DEFINES 
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define CHASSIS_WHEEL_X_LENGTH (0.40f)      // meters
#define CHASSIS_WHEEL_Y_LENGTH (0.35f)      // meters
#define CHASSIS_OMNI_WHEEL_RADIUS (0.076f)  // meters
#define CHASSIS_MOTOR_DEC_RATIO (19.0f)     // motor deduction ratio 19:1
#define CHASSIS_SLEF_GYRO_ANG_VEL 80
#define CHASSIS_MAX_SPEED 16834

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
#ifdef MECANUM_GIMBAL
#define YAW_ECD_CENTER 3350
#define PITCH_ECD_CENTER 2035
#elif OMNI_GIMBAL
#define YAW_ECD_CENTER 1025
#define PITCH_ECD_CENTER 4800
#elif SWERVE_GIMBAL
#define YAW_ECD_CENTER 0
#define PITCH_ECD_CENTER 0
#else
#define YAW_ECD_CENTER 0
#define PITCH_ECD_CENTER 0
#endif

#define PITCH_ECD_DELTA 1364  //60/180*4096
#define PITCH_GEAR_RATIO 1    // The ratio of the gear box of the pitch motor
#define PITCH_GYRO_DELTA (20.0f * DEGREE2RAD * PITCH_GEAR_RATIO)

#define YAW_GEAR_RATIO 1.0f  //if install a gear, calc the gear ratio here
#define YAW_POSITIVE_DIR \
    -1  //since we map the ecd (0,8192) to (-pi,pi), the output of first pid controller would \
        //posiibly is turned to negative value, we need to calibrate the correct direction    \
        //of this changed output for speed controller

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

#define SERVO_PWM_CLOSE_LID 366  //clockwise 120 degree
#define SERVO_PWM_OPEN_LID \
    170  //counter-clockwise 120 degree \
         //for 90 degree, turn open lid value sto 110
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
#define SHOOT_LOADER_2006_INDEX 2

/* =========================================================================
 * COMM DEFINES
 * ====================================================================== */
/* Define used can id */
//higher id, lower priority
//FIXME: adjust the priority
#define IDLE_COMM_ID 0x300
#define ANGLE_COMM_ID 0x301
#define RC_COMM_ID 0x302
#define PC_COMM_ID 0x303
#define KEY_COMM_ID 0x304
#define REF_COMM_ID 0x305
#define PC_EXT_KEY_ID 0x306
#define TOTAL_COMM_ID 7

#define ANGLE_IDX (ANGLE_COMM_ID - IDLE_COMM_ID)
#define RC_IDX (RC_COMM_ID - IDLE_COMM_ID)
#define PC_IDX (PC_COMM_ID - IDLE_COMM_ID)
#define REF_IDX (REF_COMM_ID - IDLE_COMM_ID)
#define KEY_IDX (KEY_COMM_ID - IDLE_COMM_ID)
#define PC_EXT_KEY_IDX (PC_EXT_KEY_ID - IDLE_COMM_ID)

#define ANGLE_COMM_SCALE_FACTOR \
    (32767.0f / PI -            \
     500.0f)  //32767 is the maximum size of int16_t														 // exp: since we need to transmit the float angle(-pi, pi), we need to transfer it														 // to int16_t, and rescale it in the receiver side.

#endif

/* =========================================================================
 * TIMER DEFINES
 * ====================================================================== */
// #define DISABLE_MOTOR_SEND

/* Referee App Defines */
