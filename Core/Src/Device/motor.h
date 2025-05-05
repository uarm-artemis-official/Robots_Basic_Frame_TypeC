/*******************************************************************************
* @file           : motor.h
* @created time	  : Mar, 2021
* @creator        : Bailiang
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef MOTOR_H_
#define MOTOR_H_

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "public_defines.h"
#include "pid.h"
#include "feedforward.h"

#include "math.h"
#include "string.h"
#include "device_types.h"

//Define needs to go ahead of include here for whatever reason....


void motor_data_init(Motor_t *motor);
void motor_init(Motor_t *motor, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf);

void parse_motor_feedback(const uint8_t *can_data, Motor_Feedback_t *motor_feedback);

void set_motor_voltage(CAN_HandleTypeDef* hcan, int32_t id, int32_t d1, int32_t d2, int32_t d3, int32_t d4);
void set_motor_can_current(int32_t v1, int32_t v2, int32_t v3, int32_t v4, int32_t control_indicator);

float calc_shoot_mag_dual_pid(float target_value, float f_cur_val, float s_cur_val, int motor_idx, float dt);
//void set_motor_can_volt(int32_t *motor_tx_buffer, float a1, float a2, int32_t v3, int32_t v4, int32_t control_indicator, GimbalMotorMode_t mode, uint8_t idle_flag, Gimbal_t *gimbal);
void Motor_pid_set_angle(Motor_t* motor, double angle, double p, double i, double d, int is_Pitch);
void Motor_set_raw_value(Motor_t* motor, double value);

/* LK motor */
void LK_motor_init(uint8_t lk_motor_id);
void LK_motor_send(CAN_HandleTypeDef* hcan, uint32_t id, LK_Motor_Command_t control_cmd, int32_t sendValue);

/* debug mode */
void motor_debug_init(uint8_t motor_id, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf);
void set_motor_debug_can_volt(float a1, float a2, int32_t v3, int32_t v4, int32_t control_indicator, GimbalMotorMode_t mode);

#endif /* SRC_DEVICE_MOTOR_H_ */
