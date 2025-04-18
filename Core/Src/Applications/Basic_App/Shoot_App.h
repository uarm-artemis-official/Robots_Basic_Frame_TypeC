/*******************************************************************************
* @file           : Shoot_App.h
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SHOOT_APP_H__
#define __SHOOT_APP_H__

#include "string.h"
#include "tim.h"
#include "public_defines.h"
#include "buzzer.h"
#include "motor.h"
#include "ramp.h"
#include "message_center.h"

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

/* define user structure here */
/**
  * @brief  shoot task main struct
  */
typedef enum{
	OPEN = 0,
	CLOSE
}ShootLidStatus_t;


typedef struct{
	float mag_tar_spd;
	float mag_tar_angle; //target relative angle refered to cur_abs_position
	float mag_cur_angle; //current actual relative angle ahs been reached
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
}Shoot_t;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

/* define user created variables here */



/* functions declaration here */
void Shoot_Task_Func(void const * argument);
void shoot_task_init(Shoot_t *sht, Motor_t *s_motors);
void shoot_firc_init(Shoot_t *sht);
void shoot_params_init(Shoot_t *sht);
void shoot_servo_init(void);
void shoot_fric_pwm_engagement(Shoot_t *sht, uint16_t target_pwm);
void shoot_fric_can_engagement(Shoot_t *sht, Motor_t *s_motors, uint16_t target_can);
void set_shoot_mode(Shoot_t *sht, ShootActMode_t mode);
void set_lid_status(Shoot_t *sht, ShootLidStatus_t status);
void set_mag_motor_speed(Shoot_t *sht, float spd);
void set_mag_motor_angle(Shoot_t *sht, float tar_angle);
void set_fric_motor_speed(Shoot_t *sht, int16_t spd);
void set_fric_motor_current(Shoot_t *sht, int16_t spd);
void set_servo_value(uint16_t pwm_value);
void shoot_get_motor_feedback(Shoot_t *shoot, Motor_t *s_motors);
void shoot_calc_loader_pid_out(Shoot_t *sht, Motor_t *s_motors);
void shoot_calc_fric_pid_out(Motor_t *motor, float target);
void shoot_send_motor_volts(Motor_t *s_motors);
void shoot_mag_get_rel_angle(Shoot_t *sht);
void shoot_detect_mag_status(Shoot_t *sht);
void shoot_stop(Shoot_t *sht);
void shoot_execute(Shoot_t *sht, Motor_t *s_motors);
void shoot_get_rc_info(Shoot_t *shoot);
int16_t shoot_mag_update_turns(Shoot_t *sht, int16_t raw_ecd, int16_t prev_ecd);



#endif /* __SHOOT_APP_H__ */
