/*******************************************************************************
* @file           : public_defines.h
* @brief          : Defines any specific variables shared by the project.
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __PUBLIC_DEFINES_H__
#define __PUBLIC_DEFINES_H__

/******* motor public defines *******/
#define KP_SWERVE_DRIVE 5
#define KI_SWERVE_DRIVE 0
#define KD_SWERVE_DRIVE 0
#define BETA_SWERVE_DRIVE 1
#define YETA_SWERVE_DRIVE 0
#define MIN_OUT_SWERVE_DRIVE -2000
#define MAX_OUT_SWERVE_DRIVE 2000

/* motor pid param */
// TODO: Move PID parameters into separate header file.

// Chassis spin pid
#define KP_CHASSIS_SPIN 1
#define KI_CHASSIS_SPIN 0
#define KD_CHASSIS_SPIN 0
#define BETA_CHASSIS_SPIN 1
#define YETA_CHASSIS_SPIN 0

/* wheels 3508 single loop control */
#define kp_wheel 5
#define ki_wheel 0
#define kd_wheel 0
#define beta_wheel 1
#define yeta_wheel 0

#define max_out_wheel 5000  // not tuned yet
#define max_I_out_wheel 0
#define max_err_wheel 5000

#ifdef TUNE_GIMBAL_PID
#define kp_angle_yaw 0
#define ki_angle_yaw 0
#define kd_angle_yaw 0
#define beta_angle_yaw 1
#define yeta_angle_yaw 0
#define max_out_angle_yaw 800
#define max_I_out_angle_yaw 1
#define max_err_angle_yaw 100

#define kp_spd_yaw 0
#define ki_spd_yaw 0
#define kd_spd_yaw 0
#define beta_spd_yaw 1
#define yeta_spd_yaw 1
#define max_out_spd_yaw 20000
#define max_I_out_spd_yaw 1000
#define max_err_spd_yaw 5000
#define kf_spd_yaw 0

#define kp_angle_pitch 0
#define ki_angle_pitch 0
#define kd_angle_pitch 0
#define beta_angle_pitch 1
#define yeta_angle_pitch 1
#define max_out_angle_pitch 1000
#define max_I_out_angle_pitch 0
#define max_err_angle_pitch 100

#define kp_spd_pitch 0
#define ki_spd_pitch 0
#define kd_spd_pitch 0
#define beta_spd_pitch 1
#define yeta_spd_pitch 1
#define max_out_spd_pitch 30000
#define max_I_out_spd_pitch 3000
#define max_err_spd_pitch 5000
#define kf_spd_pitch 0
#else
/* gimbal 6020 dual loop control */
#define kp_angle_yaw 150  // 200
#define ki_angle_yaw 0
#define kd_angle_yaw 0  // 15
#define beta_angle_yaw 1
#define yeta_angle_yaw 0
#define max_out_angle_yaw 800  // not tuned yet
#define max_I_out_angle_yaw 1
#define max_err_angle_yaw 100  //2.0f*PI

#define kp_spd_yaw 400  // consider going to 500 (600)
#define ki_spd_yaw 70   //0.1 (80)
#define kd_spd_yaw 0    //0.5
#define beta_spd_yaw 1
#define yeta_spd_yaw 1
#define max_out_spd_yaw 20000  // not tuned yet
#define max_I_out_spd_yaw 1000
#define max_err_spd_yaw 5000
#define kf_spd_yaw 0

#define kp_angle_pitch 300
#define ki_angle_pitch 30
#define kd_angle_pitch 0
#define beta_angle_pitch 1
#define yeta_angle_pitch 0
#define max_out_angle_pitch 1000
#define max_I_out_angle_pitch 0
#define max_err_angle_pitch 100  //2.0f*PI

#define kp_spd_pitch 180
#define ki_spd_pitch 0
#define kd_spd_pitch 0
#define beta_spd_pitch 1
#define yeta_spd_pitch 0
#define max_out_spd_pitch 30000
#define max_I_out_spd_pitch 3000
#define max_err_spd_pitch 5000
#define kf_spd_pitch 0
#endif

/* shoot 3508/2006  mag dual loop/3508 fric single loop control */
//2006 mag settings
#define kp_spd_mag_2006 5
#define ki_spd_mag_2006 0
#define kd_spd_mag_2006 0
#define beta_spd_mag_2006 1
#define yeta_spd_mag_2006 0
#define min_out_spd_mag_2006 -1000
#define max_out_spd_mag_2006 1000
// #define min_out_spd_mag_2006 -10000
// #define max_out_spd_mag_2006 10000

//3508 mag settings, not tuned yet
// #define kp_angle_mag_3508 5
// #define ki_angle_mag_3508 0
// #define kd_angle_mag_3508 0
// #define max_out_angle_mag_3508 5000  // not tuned yet
// #define max_I_out_angle_mag_3508 0
// #define max_err_angle_mag_3508 PI * 19  // mag gear ratio=19

// #define kp_spd_mag_3508 2.5
// #define ki_spd_mag_3508 0.01
// #define kd_spd_mag_3508 0
// #define max_out_spd_mag_3508 9999  // not tuned yet
// #define max_I_out_spd_mag_3508 500
// #define max_err_spd_mag_3508 5000

//fric 3508 motor settings, single loop, not tuned yet
#define kp_spd_fric 20
#define ki_spd_fric 0.3
#define kd_spd_fric 0.1
#define beta_spd_fric 1
#define yeta_spd_fric 0
#define min_out_spd_fric -1000
#define max_out_spd_fric 1000
// #define max_out_spd_fric 15000

#endif /*__PUBLIC_DEFINES_H__*/
