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


#include "usart.h"
#include "pid.h"
#include "math.h"
#include <string.h>
#include "public_defines.h"
#include "cmsis_os.h"
#include "can.h"
#include "pid.h"
#include "feedforward.h"

//Define needs to go ahead of include here for whatever reason....

#define CAN_RX_ID_START 0x201
#define MOTOR_3508_STDID 0x200
#define MOTOR_6020_STDID 0x1FF
#define MOTOR_2006_STDID 0x1FF

#define CHASSIS_MOTOR_MAX_DELTA 1000
#define GIMBAL_MOTOR_MAX_DELTA 1000
#define CHASSIS_MOTOR_PID_MAX_OUT 3500
#define GIMBAL_MOTOR_PID_MAX_OUT 2000
#define FRIC_MOTOR_PID_MAX_VALUE 2000
#define MAG_MOTOR_PID_MAX_VALUE 10000

/*
 * @brief Structure for all motors installed
 * @Note  first 4 motors will use 0x200, last 4 motors will use 0x1FF
 * 		    motor_data[0]: chassis 	3508
 * 			motor_data[1]: chassis 	3508
 *			motor_data[2]: chassis 	3508
 *			motor_data[3]: chassis  3508
 *			motor_data[4]: gimbal 	6020 pitch
 *			motor_data[5]: gimbal 	6020 yaw
 *			motor_data[6]: magazine 2006
 * */

/**
  * @brief basic gimbal struct
  */
typedef struct {
	uint32_t stdid;
	PID_t f_pid; //first pid handler for single-loop control
	PID_t s_pid; //second pid handler for dual-loop control
	FeedForward_t ff;
} Motor_Info;


typedef struct {
	Motor_Info motor_info;
	Motor_Feedback_t motor_feedback;
	//Data need to sent to Motor
	int32_t tx_data;
} Motor_t;
/*
 * @brief Structure for all motors installed
 * @Note  first 4 motors will use 0x200, last 4 motors will use 0x1FF
 * 		    motor_data[0]: chassis 	3508/shoot fric left  3508
 * 			motor_data[1]: chassis 	3508/shoot fric right 3508
 *			motor_data[2]: chassis 	3508/shoot magazine 3508
 *			motor_data[3]: chassis  3508
 *			motor_data[4]: gimbal 	6020 pitch
 *			motor_data[5]: gimbal 	6020 yaw
 *			motor_data[6]: magazine 2006
 * */

void motor_data_init(Motor_t *motor);
void motor_init(Motor_t *motor, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf);

void parse_motor_feedback(const uint8_t *can_data, Motor_Feedback_t *motor_feedback, uint8_t size);

void set_motor_voltage(CAN_HandleTypeDef* hcan, int32_t id, int32_t d1, int32_t d2, int32_t d3, int32_t d4);
void set_motor_can_current(int32_t v1, int32_t v2, int32_t v3, int32_t v4, int32_t control_indicator);

float calc_shoot_mag_dual_pid(float target_value, float f_cur_val, float s_cur_val, int motor_idx, float dt);
//void set_motor_can_volt(int32_t *motor_tx_buffer, float a1, float a2, int32_t v3, int32_t v4, int32_t control_indicator, GimbalMotorMode_t mode, uint8_t idle_flag, Gimbal_t *gimbal);
void Motor_pid_set_angle(Motor_t* motor, double angle, double p, double i, double d, int is_Pitch);
void Motor_set_raw_value(Motor_t* motor, double value);

/* debug mode */
void motor_debug_init(uint8_t motor_id, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf);
void set_motor_debug_can_volt(float a1, float a2, int32_t v3, int32_t v4, int32_t control_indicator, GimbalMotorMode_t mode);

#endif /* SRC_DEVICE_MOTOR_H_ */
