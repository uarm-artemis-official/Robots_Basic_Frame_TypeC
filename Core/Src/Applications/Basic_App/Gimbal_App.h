/*******************************************************************************
* @file           : Gimbal_App.h
* @brief          : gimbal task managing 2 gimbal motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __GIMBAL_APP_H__
#define __GIMBAL_APP_H__

#include <string.h>

#include "imu.h"
#include <motor.h>
#include <pack_handler.h>
#include "ramp.h"
#include "maths.h"
#include "kalman_filters.h"
#include "message_center.h"
#include "debugger.h"
#include "public_defines.h"
#include "angle_process.h"
#include "dwt.h"
#include "task.h"
#include "gpio.h"

/*
 * @attention:
 * 		We transit each angle mentioned here to radian because:
 * 		1. Unit and normalize the calculation of PID (spec for angular loop)
 * 		2. for safety consideration, sometimes swap the mode between gyro and ecd
 * 		   keep radians helping unit the input value
 * 		3. easier for calculating cos/sin functions
 *
 * */

/* define general declarations for gimbal task here */
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

/* define user structure here */

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

	int16_t yaw_ecd_center;			//center position of the yaw motor by encoder
	int16_t pitch_ecd_center;		//center position of the pitch motor by encoder
	float yaw_imu_center;
	float pitch_imu_center;

	PID2_t yaw_f_pid;
	PID2_t yaw_s_pid;
	PID2_t pitch_f_pid;
	PID2_t pitch_s_pid;

	Gimbal_Axis_t axis;

//	Motor_Feedback_t yaw_ecd_fb;	//yaw feedback data pool
//	Motor_Feedback_t pitch_ecd_fb; //pitch feedback data pool

	/* algorithm related */
	ramp_t yaw_ramp;		  // yaw ramp for calibration process
	ramp_t pitch_ramp;		  // pitch ramp for calibration process
	AhrsSensor_t ahrs_sensor; // copy the sensor data from imu
	Attitude_t euler_angle;   // quaternion to euler's angle

	/* filters */
	/* pc control filters */
	ewma_filter_t ewma_f_x;  //Exponential mean filtering for yaw
	ewma_filter_t ewma_f_y;	 //Exponential mean filtering for pitch
//	sliding_mean_filter_t swm_f_x; //Sliding window mean filter for yaw
//	sliding_mean_filter_t swm_f_y; //Sliding window mean filter for pitch
	/* auto aimming */
	ewma_filter_t ewma_f_aim_yaw;
	ewma_filter_t ewma_f_aim_pitch;

	first_order_low_pass_t folp_f_yaw; //first order low pass filter for imu data
	first_order_low_pass_t folp_f_pitch; //first order low pass filter for imu data;
	kalman_filter_t kalman_f;// first order gyroscope kalman filter for imu data

	GimbalMotorMode_t gimbal_motor_mode;  //gyro or encoder
	GimbalMotorMode_t prev_gimbal_motor_mode;  //gyro or encoder
	BoardActMode_t gimbal_act_mode;		  //gimbal center, gimbal follow, etc
	BoardActMode_t prev_gimbal_act_mode;
	BoardMode_t gimbal_mode;			  //idle(safe) or normal

}Gimbal_t;

/* define user created variables here */

/* functions declaration here */
void Gimbal_Task_Function(void const * argument);
void gimbal_task_init(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_reset_data(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_calibration_reset(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_get_rc_info(Gimbal_t *gbal);
void gimbal_get_motor_feedback(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_get_imu_headings(Gimbal_t *gbal);

void gimbal_set_modes(Gimbal_t* gbal, uint8_t modes[3]);
void gimbal_set_board_mode(Gimbal_t *gbal, BoardMode_t mode);
void gimbal_set_act_mode(Gimbal_t *gbal, BoardActMode_t mode);
void gimbal_set_motor_mode(Gimbal_t *gbal, GimbalMotorMode_t mode);
void gimbal_safe_mode_switch(Gimbal_t *gbal);
// gyro base functions
void gimbal_get_raw_mpu_data(Gimbal_t *gbal, IMU_t *imu_hldr);
void gimbal_get_euler_angle(Gimbal_t *gbal);
void gimbal_update_imu_angle(Gimbal_t *gbal, float yaw, float pitch);
// ecd base fucntions
//void gimbal_get_ecd_fb_data(Gimbal_t *gbal);
int16_t gimbal_calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);
void gimbal_update_ecd_euler_angle(Gimbal_t *gbal, float yaw_target_angle, float pitch_target_angle);
void gimbal_update_ecd_angles(Gimbal_t *gbal, Motor_t *g_motors);
//public functions
void gimbal_update_truns(Gimbal_t *gbal, float halfc);
void gimbal_set_angle(Gimbal_t *gbal, float target_angle);
void gimbal_set_limited_angle(Gimbal_t *gbal, float yaw_target_angle, float pitch_target_angle);
void gimbal_set_spd(Gimbal_t *gbal, int16_t yaw_target_spd);
void gimbal_cmd_exec(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_update_rel_turns(Gimbal_t* gbal, int jump_threshold);
void gimbal_send_rel_angles(Gimbal_t *gbal);
void gimbal_calc_rel_targets(Gimbal_t *gbal, float delta_yaw, float delta_pitch);
void gimbal_calc_channels_to_angles(const int16_t *g_channels, float deltas[2]);
void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, UC_Auto_Aim_Pack_t *pack);

float gimbal_calc_dual_pid_out(PID2_t *f_pid, PID2_t *s_pid, float f_cur_val);
void gimbal_send_motor_volts(Motor_t *g_motors);

void gimbal_update_headings(Gimbal_t *gbal, Motor_t *g_motors);
void gimbal_update_targets(Gimbal_t *gbal, int16_t *g_channels);

float calc_rel_angle(float angle1, float angle2);

#endif /* __SRC_APPLICATIONS_GIMBAL_APP_H_ */
