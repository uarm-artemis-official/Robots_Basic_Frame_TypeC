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
#include "event_center.h"
#include "debugger.h"
#include "public_defines.h"
#include "angle_process.h"
#include "dwt.h"
#include "task.h"
#include "gpio.h"
#include "apps_types.h"

/*
 * @attention:
 * 		We transit each angle mentioned here to radian because:
 * 		1. Unit and normalize the calculation of PID (spec for angular loop)
 * 		2. for safety consideration, sometimes swap the mode between gyro and ecd
 * 		   keep radians helping unit the input value
 * 		3. easier for calculating cos/sin functions
 *
 * */

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
