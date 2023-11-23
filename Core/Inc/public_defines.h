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
#ifndef PI
  #define PI 		 3.1415926f
  #define DEGREE2RAD 0.0174533f
  #define RAD2DEGEE  57.3f
  #define ECD2RAD    ((2.0*PI)/ 8192.0f)
  #define ECD2DEGREE ( 360f  / 8192.0f)
#endif

#define MAX_CAN_MOTOR_NUM 8

/* motor can id */
#define CHASSIS_ECD_CONST_OMEGA 120
#define max_wheel_num 4
#define wheel_id1 0
#define wheel_id2 1
#define wheel_id3 2
#define wheel_id4 3

#define yaw_id 4
#define pitch_id 5

#define fric_left_id 0
#define fric_right_id 1
#define mag_2006_id 6
#define mag_3508_id 2

/* motor pid param */
/* wheels 3508 single loop control */
#define kp_wheel 5
#define ki_wheel 0
#define kd_wheel 0
#define max_out_wheel 5000 // not tuned yet
#define max_I_out_wheel 0
#define max_err_wheel 5000

/* gimbal 6020 dual loop control */
#define kp_angle_yaw 550//800
#define ki_angle_yaw 0.1
#define kd_angle_yaw 0
#define max_out_angle_yaw 5000 // not tuned yet
#define max_I_out_angle_yaw 0
#define max_err_angle_yaw 500//2.0f*PI

#define kp_spd_yaw 385
#define ki_spd_yaw 0.1
#define kd_spd_yaw 0.5
#define max_out_spd_yaw 15000 // not tuned yet
#define max_I_out_spd_yaw 3000
#define max_err_spd_yaw 5000
#define kf_spd_yaw 0

#define kp_angle_pitch 1000
#define ki_angle_pitch 0.1
#define kd_angle_pitch 0
#define max_out_angle_pitch 5000 // not tuned yet
#define max_I_out_angle_pitch 0
#define max_err_angle_pitch 100//2.0f*PI

#define kp_spd_pitch 385
#define ki_spd_pitch 0.2
#define kd_spd_pitch 120
#define max_out_spd_pitch 15000 // not tuned yet
#define max_I_out_spd_pitch 3000
#define max_err_spd_pitch 5000
#define kf_spd_pitch 0

/* shoot 3508/2006  mag dual loop/3508 fric single loop control */
//2006 mag settings
#define kp_angle_mag_2006 1000
#define ki_angle_mag_2006 0
#define kd_angle_mag_2006 0
#define max_out_angle_mag_2006 5000 // not tuned yet
#define max_I_out_angle_mag_2006 0
#define max_err_angle_mag_2006 PI*19 // mag gear ratio=19

#define kp_spd_mag_2006 3
#define ki_spd_mag_2006 0.01
#define kd_spd_mag_2006 0
#define max_out_spd_mag_2006 9999 // not tuned yet
#define max_I_out_spd_mag_2006 500
#define max_err_spd_mag_2006 5000
#define kf_spd_mag_2006 1

//3508 mag settings, not tuned yet
#define kp_angle_mag_3508 1000
#define ki_angle_mag_3508 0
#define kd_angle_mag_3508 0
#define max_out_angle_mag_3508 5000 // not tuned yet
#define max_I_out_angle_mag_3508 0
#define max_err_angle_mag_3508 PI*19 // mag gear ratio=19

#define kp_spd_mag_3508 2.5
#define ki_spd_mag_3508 0.01
#define kd_spd_mag_3508 0
#define max_out_spd_mag_3508 9999 // not tuned yet
#define max_I_out_spd_mag_3508 500
#define max_err_spd_mag_3508 5000
#define kf_spd_mag_3508 100

//fric 3508 motor settings, single loop, not tuned yet
#define kp_spd_fric 10
#define ki_spd_fric 0.5
#define kd_spd_fric 0.5
#define max_out_spd_fric 10000// not tuned yet
#define max_I_out_spd_fric 500
#define max_err_spd_fric 5000

//fric 2305 PWM settings, open loop
#define fric_min_pwm_speed 1000
#define fric_max_pwm_speed 2000

/******* referee system limit defines *******/
/* chassis power */
#define chassis_l1_power 60
#define chassis_l2_power 80
#define chassis_l3_power 100

#define chassis_l1_x_speed 300 //max 660
#define chassis_l2_x_speed 600 //max 660
#define chassis_l3_x_speed 660 //max 660

#define chassis_l1_y_speed 600 //max 660
#define chassis_l2_y_speed 600 //max 660
#define chassis_l3_y_speed 660 //max 660

#define chassis_l1_w_speed 350 //max 660
#define chassis_l2_w_speed 500 //max 660
#define chassis_l3_w_speed 660 //max 660

#endif /*__PUBLIC_DEFINES_H__*/
