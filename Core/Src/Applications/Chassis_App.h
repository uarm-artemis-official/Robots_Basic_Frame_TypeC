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

#include "motor.h"
#include "apps_types.h"

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
