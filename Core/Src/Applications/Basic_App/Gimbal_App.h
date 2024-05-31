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

#include <auto_aim_pack.h>
#include <imu.h>
#include <motor.h>
#include "ramp.h"
#include "maths.h"
#include "kalman_filters.h"

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
//#define MANUAL_SET_GIMBAL_MODES

#define PITCH_ECD_CENTER  3300//6100 manually measured data
#define PITCH_ECD_DELTA  1364  //60/180*4096
#define PITCH_GYRO_DELTA (80.0f * DEGREE2RAD)
#define PITCH_GEAR_RATIO 4 // The ratio of the gear box of the pitch motor
#define YAW_ECD_CENTER 620//300

#define YAW_GEAR_RATIO 1.0f		//if install a gear, calc the gear ratio here
#define YAW_POSITIVE_DIR 1    //since we map the ecd (0,8192) to (-pi,pi), the output of first pid controller would
								//posiibly is turned to negative value, we need to calibrate the correct direction
								//of this changed output for speed controller

#define GIMBAL_INIT_TIME_MS 1000  	// init delay duration in mili-second
#define TGT_CONST 100000			// after-detection delay


/* define user structure here */
typedef struct{
	float vx;
	float vy;
	float wz;
}Gimbal_Axis_t; //for remote controller set gimbal dir

typedef struct{
	/* gimbal speed related */
	float yaw_ang_rate;				//angular velocity in degree/s, not used
	float pitch_ang_rate;
	float yaw_speed_rate;			//rpm in m/s, not used
	float pitch_speed_rate;			//rpm in m/s, not used

	/* gimbal position related */
	float yaw_tar_angle;			//yaw angle target angle in radians
	float yaw_cur_abs_angle;		//yaw current absolute angle updated by gyro
	float yaw_cur_rel_angle;		//yaw current relative angle updated by encoder
	float yaw_prev_angle;			//yaw previous absolute angle for updating turns, not used
	float pitch_tar_angle;			//pitch angle target angle in radians
	float pitch_cur_abs_angle;		//pitch current absolute angle updated by gyro
	float pitch_cur_rel_angle;		//pitch current absolute angle updated by gyro
	float pitch_prev_angle;    		//pitch previous absolute angle for updating turns

	float yaw_total_turns;
	float pitch_total_turns;
	float final_abs_yaw;
	float final_abs_pitch;

	double gyro_offset_slope;    //offset func: -0.000000000184229  -0.001797605717065
								 //fit a linear func to compensate yaw abs angle shift, now abandoned
	uint32_t gyro_offset_count;  // This method has been abandoned

	Gimbal_Axis_t axis;

	/* only spd control */
	int16_t yaw_tar_spd;
	int16_t pitch_tar_spd;

	int16_t yaw_turns_count;		//turns counter, not used
	int16_t yaw_ecd_center;			//center position of the yaw motor by encoder
	int16_t pitch_ecd_center;		//center position of the pitch motor by encoder

	Motor_Feedback_Data_t yaw_ecd_fb;	//yaw feedback data pool
	Motor_Feedback_Data_t pitch_ecd_fb; //pitch feedback data pool

	/* algorithm related */
	ramp_t yaw_ramp;		  // yaw ramp for calibration process
	ramp_t pitch_ramp;		  // pitch ramp for calibration process
	AhrsSensor_t ahrs_sensor; // copy the sensor data from imu
	Attitude_t euler_angle;   // quaternion to euler's angle

	/* filters */
	/* pc control filters */
	ewma_filter_t ewma_f_x;  //Exponential mean filtering for yaw
	ewma_filter_t ewma_f_y;	 //Exponential mean filtering for pitch
	sliding_mean_filter_t swm_f_x; //Sliding window mean filter for yaw
	sliding_mean_filter_t swm_f_y; //Sliding window mean filter for pitch
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
Gimbal_t gimbal;


/* extern global variables here */
extern CAN_HandleTypeDef hcan1;
extern Comm_t comm_pack;
extern Motor motor_data[MOTOR_COUNT];
extern uint8_t gimbal_cali_done_flag;
extern IMU_t imu;
extern PID_t tune_pid_f;
extern PID_t tune_pid_s;

/* define user created variables here */



/* functions declaration here */
void Gimbal_Task_Function(void const * argument);
void gimbal_task_init(Gimbal_t *gbal);
void gimbal_reset_data(Gimbal_t *gbal);
void gimbal_calibration_reset(Gimbal_t *gbal);
void gimbal_set_mode(Gimbal_t *gbal, BoardMode_t mode);
void gimbal_set_act_mode(Gimbal_t *gbal, BoardActMode_t mode);
void gimbal_set_motor_mode(Gimbal_t *gbal, GimbalMotorMode_t mode);
// gyro base functions
void gimbal_get_raw_mpu_data(Gimbal_t *gbal, IMU_t *imu_hldr);
void gimbal_get_euler_angle(Gimbal_t *gbal);
void gimbal_gyro_update_abs_angle(Gimbal_t *gbal);
//ecd base fucntions
void gimbal_get_ecd_fb_data(Gimbal_t *gbal, Motor_Feedback_Data_t *yaw_motor_fb, Motor_Feedback_Data_t *pitch_motor_fb);
int16_t gimbal_get_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);
void gimbal_update_ecd_euler_angle(Gimbal_t *gbal, float yaw_target_angle, float pitch_target_angle);
void gimbal_update_ecd_rel_angle(Gimbal_t *gbal);
//public functions
void gimbal_update_truns(Gimbal_t *gbal, float halfc);
void gimbal_set_angle(Gimbal_t *gbal, float target_angle);
void gimbal_set_limited_angle(Gimbal_t *gbal, float yaw_target_angle, float pitch_target_angle);
void gimbal_set_spd(Gimbal_t *gbal, int16_t yaw_target_spd);
void gimbal_cmd_exec(Gimbal_t *gbal, uint8_t mode);
void gimbal_safe_mode_switch(Gimbal_t *gbal);















/* *** Old declaration *** */
double angle_preprocess(Motor* motor, int16_t recieved_angle, double red_ratio, int motor_indicator);
int16_t check_angle_greater_than_max(int32_t input_angle, int16_t max_angle, int16_t min_angle);
int16_t check_angle_smaller_than_min(int32_t input_angle, int16_t max_angle, int16_t min_angle);
int16_t check_angle_out_of_range(int32_t input_angle, int16_t max_angle, int16_t min_angle);
int32_t abs_yaw;
int32_t abs_pitch;

#endif /* __SRC_APPLICATIONS_GIMBAL_APP_H_ */
