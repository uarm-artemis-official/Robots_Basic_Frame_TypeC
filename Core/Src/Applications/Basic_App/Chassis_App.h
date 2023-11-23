/*******************************************************************************
* @file           : Chassis_App.h
* @brief          : chassis task managing 4 chassis motors
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__

#include <Gimbal_App.h>
#include <Control_App.h>
#include <referee.h>


/* define general declarations for gimbal task here */
#define TGT_CONST 100000
#define CHASSIS_WHEEL_X_LENGTH  0.50f//m
#define CHASSIS_WHEEL_Y_LENGTH  0.30f//m
#define CHASSIS_MOTOR_DEC_RATIO 19.0f; //motor deduction ratio 19:1
#define CHASSIS_SLEF_GYRO_ANG_VEL 80
#define CHASSIS_MAX_SPEED 16834

/* power limit defines */
#define CHASSIS_POWER_DANGER      20 //random value, test soon
#define CHASSIS_POWER_THRESHOLD   45  //random value, test soon
#define POWER_TO_CURRENT          (1.0f) //random value, test soon

/* manual gear defines, not used */
#define NO_GEAR  0
#define GEAR_LOW 500
#define GEAR_MID 1000
#define GEAR_HIGH 2000

/* define user structure here */
/*
 * @brief basic chassis structure
 *
 * */
typedef enum{
	AUTO_GEAR = 0, //referee system up, auto-adjust chassis spd limit
	MANUAL_GEAR	   //referee system down, manual-adjust chassis spd limit
}ChassisGearMode_t;

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

	PID_t f_pid;//for Chassis twist(in Gimbal_Center mode)
	int16_t mec_spd[4];
	Gimbal_Axis_t gimbal_axis;

	uint16_t chassis_gear;
	ChassisPowerStat_t ref_power_stat;
	ChassisPowerStat_t local_power_stat;

	BoardMode_t chassis_mode;//chassis mode selection
	BoardActMode_t chassis_act_mode;
	ChassisGearMode_t chassis_gear_mode;

}Chassis_t;
// define chassis velocity structure


/* extern global variables here */
extern Referee_t referee;
extern Motor motor_data[MOTOR_COUNT];
Chassis_t chassis;

/* define user creaeted variables here */

/* functions declaration here */
void Chassis_Task_Func(void const * argument);
void chasiss_task_init(Chassis_t* chassis_hdlr);
void chassis_reset_data(Chassis_t *chassis_hdlr);
void chassis_set_mode(Chassis_t *chassis_hdlr, BoardMode_t mode);
void chassis_set_act_mode(Chassis_t *chassis_hdlr, BoardActMode_t mode);
void mecanum_wheel_calc_speed(Chassis_t *chassis_hdlr);
void chassis_update_chassis_coord(Chassis_t *chassis_hdlr, RemoteControl_t *rc_hdlr);
void chassis_update_gimbal_coord(Chassis_t *chassis_hdlr, RemoteControl_t *rc_hdlr);
void chassis_brake(float *vel, float ramp_step, float stop_threshold);
void chassis_exec_act_mode(Chassis_t *chassis_hdlr);
void chassis_execute(Chassis_t *chassis_hdlr);

/* power limit */
void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t *ref);
void chassis_power_limit_referee(Chassis_t* chassis_hdlr);
void chassis_power_limit_local(Chassis_t* chassis_hdlr, uint16_t local_power_limit);

/*back up*/
int32_t motor_move_period_ground(double patrol_vw, int32_t chassis_control_counter, Chassis_t* chassis);
#endif /* SRC_APPLICATIONS_CHASSIS_APP_H_ */
