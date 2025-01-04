/*******************************************************************************
* @file           : Chassis_App.h
* @brief          : chassis task managing 4 chassis motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__

#include "message_center.h"
#include "math.h"
#include "maths.h"
#include "arm_math.h"
#include "public_defines.h"
#include "pid.h"
#include "gpio.h"
#include "motor.h"


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


/* define user structure here */
/*
 * @brief basic chassis structure
 *
 * */
typedef enum{
	AUTO_GEAR = 0, //referee system up, auto-adjust chassis spd limit
	MANUAL_GEAR	   //referee system down, manual-adjust chassis spd limit
}ChassisGearMode_t;

typedef enum{
	 NO_GEAR = 0,
	 GEAR_LOW = 500,
	 GEAR_MID = 1000,
	 GEAR_HIGH =2000
}ChassisGearValue_t;

typedef struct{
	uint16_t current;
	float power;
	uint16_t buffer_energy;
}ChassisPowerStat_t;

typedef struct{
	float vx;//x axis velocity
	float vy;//y axis velocity
	float wz;//w axis angular velocity
	float max_vx;
	float max_vy;
	float max_wz;

	float gimbal_yaw_rel_angle;
	float gimbal_yaw_abs_angle;

	float gimbal_pitch_rel_angle;
	float gimbal_pitch_abs_angle;

	uint8_t pc_target_value;

	PID_t f_pid;//for Chassis twist(in Gimbal_Center mode)
	int16_t mec_spd[4];
	Gimbal_Axis_t gimbal_axis;

	uint16_t chassis_gear;
	uint8_t prev_robot_level;
	uint8_t cur_robot_level;
	ChassisPowerStat_t ref_power_stat;
	ChassisPowerStat_t local_power_stat;

	BoardMode_t chassis_mode;//chassis mode selection
	BoardActMode_t chassis_act_mode;
	ChassisGearMode_t chassis_gear_mode;

}Chassis_t;
// define chassis velocity structure


/* extern global variables here */


/* define user creaeted variables here */

/* functions declaration here */
void Chassis_Task_Func(void const * argument);
void chasiss_task_init(Chassis_t* chassis_hdlr);
void chassis_reset_data(Chassis_t *chassis_hdlr);
void mecanum_wheel_calc_speed(Chassis_t *chassis_hdlr);
void chassis_update_chassis_coord(Chassis_t *chassis_hdlr, int16_t *channels);
void chassis_update_gimbal_coord(Chassis_t *chassis_hdlr, int16_t *channels);
void chassis_brake(float *vel, float ramp_step, float stop_threshold);
void chassis_exec_act_mode(Chassis_t *chassis_hdlr);

void chassis_calc_wheel_pid_out(Chassis_t *chassis_hdlr, Motor_t *wheels);
void chassis_send_wheel_volts(Chassis_t *chassis_hdlr, Motor_t *wheels);
void chassis_get_rc_info(Chassis_t *chassis_hdlr, int16_t *channels);
void chassis_get_wheel_feedback(Motor_t *wheels);
void chassis_get_gimbal_rel_angles(Chassis_t *chassis_hdlr);

/* power limit */
//void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t *ref);
void chassis_power_limit_referee(Chassis_t* chassis_hdlr);
void chassis_power_limit_local(Chassis_t* chassis_hdlr, uint16_t local_power_limit);
void select_chassis_speed(Chassis_t* chassis_hdlr, uint8_t level);
//void chassis_manual_gear_set(Chassis_t* chassis_hdlr, RemoteControl_t *rc_hdlr);

/*back up*/
int32_t motor_move_period_ground(double patrol_vw, int32_t chassis_control_counter, Chassis_t* chassis);
#endif /* SRC_APPLICATIONS_CHASSIS_APP_H_ */
