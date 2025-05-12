#ifndef __APPS_CONFIG_H
#define __APPS_CONFIG_H

/* =========================================================================
 * CHASSIS DEFINES 
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define CHASSIS_WHEEL_X_LENGTH  0.50f//m
#define CHASSIS_WHEEL_Y_LENGTH  0.30f//m
#define CHASSIS_MOTOR_DEC_RATIO 19.0f; //motor deduction ratio 19:1
#define CHASSIS_SLEF_GYRO_ANG_VEL 80
#define CHASSIS_MAX_SPEED 16834

/* power limit defines */
#define CHASSIS_POWER_DANGER      20 //random value, test soon
#define CHASSIS_POWER_THRESHOLD   45  //random value, test soon
#define POWER_TO_CURRENT          (1.0f) //random value, test soon
#define CHASSIS_PC_RAMP_VALUE 	 (0.5f) //ramp value for increment of the motors

// Wheel index defines
#define CHASSIS_WHEEL1_INDEX 0
#define CHASSIS_WHEEL2_INDEX 1
#define CHASSIS_WHEEL3_INDEX 2
#define CHASSIS_WHEEL4_INDEX 3


/* =========================================================================
 * GIMBAL DEFINES
 * ====================================================================== */
//#define GIMBAL_MOTOR_DEBUG 1
#define MODE_DEBUG 1
//#define ENABLE_MANUAL_MODE_SET

#define PITCH_ECD_CENTER 4750 //manually measured data: number increase, head down
#define PITCH_ECD_DELTA  1364  //60/180*4096
#define PITCH_GEAR_RATIO 1    // The ratio of the gear box of the pitch motor
#define PITCH_GYRO_DELTA (20.0f * DEGREE2RAD * PITCH_GEAR_RATIO) 

#define YAW_ECD_CENTER 3350
#define YAW_GEAR_RATIO 1.0f		 //if install a gear, calc the gear ratio here
#define YAW_POSITIVE_DIR -1      //since we map the ecd (0,8192) to (-pi,pi), the output of first pid controller would
								 //posiibly is turned to negative value, we need to calibrate the correct direction
								 //of this changed output for speed controller
#define GIMBAL_INIT_TIME_MS 1000 // init delay duration in mili-second
#define GIMBAL_JUMP_THRESHOLD 5.6f

#define GIMBAL_YAW_MOTOR_INDEX 0
#define GIMBAL_PITCH_MOTOR_INDEX 1
#define GIMBAL_MOTOR_COUNT 2

/* =========================================================================
 * IMU DEFINES
 * ====================================================================== */
#define IMU_TMP_PWM_HTIM    htim10
#define IMU_TMP_PWM_CHANNEL  TIM_CHANNEL_1

/* =========================================================================
 * SHOOT DEFINES
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define USE_CAN_FRIC 1//if use 3508 instead of pwm-based fric wheel motor

//Max input value (abs) for magazine motor, for p2006, it is 10000, f
//or 3508 it is 16000, for 6020 it is 30000
#define MAX_PWM_ON_TIME 2000
#define MIN_PWM_ON_TIME 1000

#define SERVO_PWM_CLOSE_LID 366 //clockwise 120 degree
#define SERVO_PWM_OPEN_LID  170	//counter-clockwise 120 degree
								//for 90 degree, turn open lid value sto 110
/* 2305 can value*/
#define LEVEL_ONE_PWM 300

#define FRIC_PWM_DELAY 10
#define FRIC_CAN_RAMP_DELAY 40

/* 3508 can value*/
//#define LEVEL_ONE_CAN_SPD 7500 // 28: 7500, 27:7350
#define LEVEL_ONE_CAN_SPD 7500

#define SHOOT_ONCE_MAG_ANGLE (20.0f * DEGREE2RAD)
#define SHOOT_CONT_MAG_SPEED 1.0*PI //rpm/sec
#define SHOOT_MAG_GEAR_RATIO 19
#define SHOOT_REVERSE_MAG_SPEED 0.2*PI


#define SHOOT_LEFT_FRIC_WHEEL_INDEX 0
#define SHOOT_RIGHT_FRIC_WHEEL_INDEX 1
#define SHOOT_LOADER_2006_INDEX 2

#endif