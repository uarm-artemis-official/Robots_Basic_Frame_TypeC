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
#define PI 3.1415926f
#define DEGREE2RAD 0.0174533f
#define RAD2DEGEE 57.3f
#define ECD2RAD ((2.0 * PI) / 8192.0f)
#define ECD2DEGREE (360f / 8192.0f)
#endif
#define MAX_CAN_MOTOR_NUM 8
#define MOTOR_COUNT 8

#define MOTOR_TX_BUFFER_SIZE 8

#define CHANNEL_OFFSET_MAX_ABS_VAL 660

/* motor can id */
#define CHASSIS_ECD_CONST_OMEGA 120
#define CHASSIS_MAX_WHEELS 4

// These are used for indexing the motor feedback array from MOTOR_READ topic.
// Formula: CAN ID = Chassis Motor CAN Std. IDs - CAN_RX_ID_START.
#define CHASSIS_WHEEL1_CAN_ID 0
#define CHASSIS_WHEEL2_CAN_ID 1
#define CHASSIS_WHEEL3_CAN_ID 2
#define CHASSIS_WHEEL4_CAN_ID 3

// These are used for indexing the motor feedback array from MOTOR_READ topic.
// Formula: CAN ID = Gimbal Motor CAN Std. IDs - CAN_RX_ID_START.
#define SHOOT_LEFT_FRIC_CAN_ID 0
#define SHOOT_RIGHT_FRIC_CAN_ID 1
#define SHOOT_LOADER_CAN_ID 6
#define GIMBAL_YAW_CAN_ID 4
#define GIMBAL_PITCH_CAN_ID 5
//#define MAG_3508_ID 2

#define TIMER_GIMBAL_MAG_ENABLE
#define TIMER_CHASSIS_WHEELS_ENABLE
//#define TIMER_FRIC_WHEELS_ENABLE
//#define TUNE_GIMBAL_PID

/* motor pid param */
// TODO: Move PID parameters into separate header file.
/* wheels 3508 single loop control */
#define kp_wheel 5
#define ki_wheel 0
#define kd_wheel 0
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

#define kp_angle_pitch 0
#define ki_angle_pitch 0
#define kd_angle_pitch 0
#define beta_angle_pitch 1
#define yeta_angle_pitch 1
#define max_out_angle_pitch 1000
#define max_I_out_angle_pitch 0
#define max_err_angle_pitch 100  //2.0f*PI

#define kp_spd_pitch 0
#define ki_spd_pitch 0
#define kd_spd_pitch 0
#define beta_spd_pitch 1
#define yeta_spd_pitch 1
#define max_out_spd_pitch 30000
#define max_I_out_spd_pitch 3000
#define max_err_spd_pitch 5000
#define kf_spd_pitch 0
#endif

/* shoot 3508/2006  mag dual loop/3508 fric single loop control */
//2006 mag settings
#define kp_angle_mag_2006 900
#define ki_angle_mag_2006 0
#define kd_angle_mag_2006 0
#define max_out_angle_mag_2006 10000  // not tuned yet
#define max_I_out_angle_mag_2006 0
#define max_err_angle_mag_2006 PI * 19  // mag gear ratio=19

#define kp_spd_mag_2006 15
#define ki_spd_mag_2006 0.01
#define kd_spd_mag_2006 0
#define max_out_spd_mag_2006 9999  // not tuned yet
#define max_I_out_spd_mag_2006 500
#define max_err_spd_mag_2006 5000
#define kf_spd_mag_2006 1

//3508 mag settings, not tuned yet
#define kp_angle_mag_3508 1000
#define ki_angle_mag_3508 0
#define kd_angle_mag_3508 0
#define max_out_angle_mag_3508 5000  // not tuned yet
#define max_I_out_angle_mag_3508 0
#define max_err_angle_mag_3508 PI * 19  // mag gear ratio=19

#define kp_spd_mag_3508 2.5
#define ki_spd_mag_3508 0.01
#define kd_spd_mag_3508 0
#define max_out_spd_mag_3508 9999  // not tuned yet
#define max_I_out_spd_mag_3508 500
#define max_err_spd_mag_3508 5000
#define kf_spd_mag_3508 100

//fric 3508 motor settings, single loop, not tuned yet
#define kp_spd_fric 20
#define ki_spd_fric 0.3
#define kd_spd_fric 0.1
#define max_out_spd_fric 15000  // not tuned yet
#define max_I_out_spd_fric 500
#define max_err_spd_fric 10000

//fric 2305 PWM settings, open loop
#define fric_min_pwm_speed 1000
#define fric_max_pwm_speed 2000

/******* referee system limit defines *******/
/* chassis power - power focused */
#define chassis_l1_pf_power 70
#define chassis_l2_pf_power 75
#define chassis_l3_pf_power 80
#define chassis_l4_pf_power 85
#define chassis_l5_pf_power 90
#define chassis_l6_pf_power 95
#define chassis_l7_pf_power 100
#define chassis_l8_pf_power 105
#define chassis_l9_pf_power 110
#define chassis_l10_pf_power 120

/* chassis power - HP focused */
#define chassis_l1_hpf_power 55
#define chassis_l2_hpf_power 60
#define chassis_l3_hpf_power 65
#define chassis_l4_hpf_power 70
#define chassis_l5_hpf_power 75
#define chassis_l6_hpf_power 80
#define chassis_l7_hpf_power 85
#define chassis_l8_hpf_power 90
#define chassis_l9_hpf_power 100
#define chassis_l10_hpf_power 120

/* chassis max speed - power focused */
#define chassis_l1_pf_padding_speed 500   //max 660
#define chassis_l2_pf_padding_speed 350   //max 660
#define chassis_l3_pf_padding_speed 400   //max 660
#define chassis_l4_pf_padding_speed 450   //max 660
#define chassis_l5_pf_padding_speed 500   //max 660
#define chassis_l6_pf_padding_speed 550   //max 660
#define chassis_l7_pf_padding_speed 600   //max 660
#define chassis_l8_pf_padding_speed 650   //max 660
#define chassis_l9_pf_padding_speed 660   //max 660
#define chassis_l10_pf_padding_speed 660  //max 660

/* chassis max speed - HP focused */
#define chassis_l1_hpf_padding_speed 500   //max 660
#define chassis_l2_hpf_padding_speed 110   //max 660
#define chassis_l3_hpf_padding_speed 120   //max 660
#define chassis_l4_hpf_padding_speed 130   //max 660
#define chassis_l5_hpf_padding_speed 140   //max 660
#define chassis_l6_hpf_padding_speed 150   //max 660
#define chassis_l7_hpf_padding_speed 160   //max 660
#define chassis_l8_hpf_padding_speed 170   //max 660
#define chassis_l9_hpf_padding_speed 180   //max 660
#define chassis_l10_hpf_padding_speed 190  //max 660

/* chassis max spin speed - power focused */
#define chassis_l1_pf_spin_speed 300   //max 660
#define chassis_l2_pf_spin_speed 120   //max 660
#define chassis_l3_pf_spin_speed 140   //max 660
#define chassis_l4_pf_spin_speed 160   //max 660
#define chassis_l5_pf_spin_speed 180   //max 660
#define chassis_l6_pf_spin_speed 200   //max 660
#define chassis_l7_pf_spin_speed 220   //max 660
#define chassis_l8_pf_spin_speed 240   //max 660
#define chassis_l9_pf_spin_speed 260   //max 660
#define chassis_l10_pf_spin_speed 280  //max 660

/* chassis max spin speed - HP focused */
#define chassis_l1_hpf_spin_speed 300   //max 660
#define chassis_l2_hpf_spin_speed 220   //max 660
#define chassis_l3_hpf_spin_speed 240   //max 660
#define chassis_l4_hpf_spin_speed 260   //max 660
#define chassis_l5_hpf_spin_speed 280   //max 660
#define chassis_l6_hpf_spin_speed 300   //max 660
#define chassis_l7_hpf_spin_speed 320   //max 660
#define chassis_l8_hpf_spin_speed 340   //max 660
#define chassis_l9_hpf_spin_speed 360   //max 660
#define chassis_l10_hpf_spin_speed 400  //max 660

#endif /*__PUBLIC_DEFINES_H__*/
