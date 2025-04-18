/*******************************************************************************
* @file           : Chassis_App.c
* @brief          : chassis task managing 4 chassis motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __CHASSIS_APP_C__
#define __CHASSIS_APP_C__

#define WITH_SLIPRING
//#define CHASSIS_POWER_LIMIT

#include "Chassis_App.h"


static Motor_t chassis_wheels[4];
static Chassis_t chassis;
static int16_t rc_channels[4];

/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* Task execution time (per loop): 1ms */
void Chassis_Task_Func(void const * argument) {
	/* task LD indicator */
	HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);

	chasiss_task_init(&chassis);

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(CHASSIS_TASK_EXEC_TIME);

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	for(;;) {
		// memcpy(&temp_referee, &referee, sizeof(Referee_t));
		chassis_get_rc_info(&chassis, rc_channels);
		chassis_get_wheel_feedback(chassis_wheels);
		chassis_get_gimbal_rel_angles(&chassis);

		chassis_update_chassis_coord(&chassis, rc_channels);
		chassis_update_gimbal_coord(&chassis, rc_channels);
		// chassis_manual_gear_set(&chassis, &rc);
		chassis_exec_act_mode(&chassis);
		chassis_calc_wheel_pid_out(&chassis, chassis_wheels);

		chassis_send_wheel_volts(&chassis, chassis_wheels);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/*
 * @brief     the initialization process of the chassis task,
 * @param[in] chassis: main chassis handler
 * */
void chasiss_task_init(Chassis_t* chassis_hdlr){
	/* set pid parameters for chassis motors */
	for(int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
		motor_init(&(chassis_wheels[i]), max_out_wheel,  max_I_out_wheel, max_err_wheel, kp_wheel, ki_wheel, kd_wheel,
					0, 0, 0, 0, 0, 0,//no second loop
					0);//spd ff gain
		motor_data_init(&(chassis_wheels[i]));
	}
	pid_param_init(&(chassis_hdlr->f_pid), 8000, 500, 5000, 600, 0.1, 100); // chassis twist pid init
	/* set initial chassis mode to idle mode or debug mode */
	chassis_hdlr->chassis_mode = IDLE_MODE;
	chassis_hdlr->chassis_act_mode = INDPET_MODE;

	chassis_hdlr->chassis_gear_mode = AUTO_GEAR;
	/* reset data */
	chassis_reset_data(chassis_hdlr);
}

/*
 * @brief 	  reset all data in the chassis main struct
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void chassis_reset_data(Chassis_t *chassis_hdlr) {
	/* init both coordinates */
	chassis_hdlr->vx = 0;
	chassis_hdlr->vy = 0;
	chassis_hdlr->wz = 0;

	chassis_hdlr->gimbal_axis.vx = 0;
	chassis_hdlr->gimbal_axis.vy = 0;
	chassis_hdlr->gimbal_axis.wz = 0;
	chassis_hdlr->gimbal_yaw_rel_angle = 0;
	chassis_hdlr->gimbal_yaw_abs_angle = 0;

	memset(&(chassis_hdlr->gimbal_axis), 0, sizeof(Gimbal_Axis_t));
	memset(&(chassis_hdlr->ref_power_stat), 0, sizeof(ChassisPowerStat_t));

	chassis_hdlr->prev_robot_level = 1; // Initalized to level 1
	chassis_hdlr->cur_robot_level = 1;
	select_chassis_speed(chassis_hdlr, chassis_hdlr->prev_robot_level);

	/* reset mecanum wheel speed */
	for(int i=0;i<4;i++)
		chassis_hdlr->mec_spd[i] = 0;

	memset(rc_channels, 0, sizeof(int16_t) * 4);
}

void swerve_drive_wheel_decomposition(Chassis_t *chassis_hdlr){
	/* Assume we already */
}


/*
 * @brief 	  Inversely calculate the mecanum wheel speed
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void mecanum_wheel_calc_speed(Chassis_t *chassis_hdlr){
	/* Assume we install the mecanum wheels as O type (also have X type), right hand define positive dir
	 *			 y length
	 *		 v1  \\ -- //  v2     <Front>		   	 A      __			wheels define:
	 *		      |    |		x length			 | vx  /		 	 		  ___
	 *		  	  |	   |                             |     \__>   wz    \\ ->    | / |
	 *		 v4	 // -- \\  v3     <Rear>     vy  < ---  						 | / |
	 *																			 |___|
	 *		--	vector([vx, vy, wz]) --
	 *	v1	=  [ vx,  vy,  wz] * (rx + ry) * motor_gearbox_ratio ->rad/s
	 *	v2  =  [-vx,  vy,  wz] * (rx + ry) * motor_gearbox_ratio
	 *	v3  =  [-vx, -vy,  wz] * (rx + ry) * motor_gearbox_ratio
	 * 	v4  =  [ vx, -vy,  wz] * (rx + ry) * motor_gearbox_ratio
	 *
	 * */
	/* X type installation */
	chassis_hdlr->mec_spd[CHASSIS_WHEEL1_CAN_ID] = (int16_t)(  chassis_hdlr->vx + chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH)*0.5) * CHASSIS_MOTOR_DEC_RATIO;
	chassis_hdlr->mec_spd[CHASSIS_WHEEL2_CAN_ID] = (int16_t)(- chassis_hdlr->vx + chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH)*0.5) * CHASSIS_MOTOR_DEC_RATIO;
	chassis_hdlr->mec_spd[CHASSIS_WHEEL3_CAN_ID] = (int16_t)(- chassis_hdlr->vx - chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH)*0.5) * CHASSIS_MOTOR_DEC_RATIO;
	chassis_hdlr->mec_spd[CHASSIS_WHEEL4_CAN_ID] = (int16_t)(  chassis_hdlr->vx - chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH)*0.5) * CHASSIS_MOTOR_DEC_RATIO;

	/* may apply super super capacity gain here */
	/* may apply level up gain and power limit here when we have referee system feedback */
}


void chassis_calc_wheel_pid_out(Chassis_t *chassis_hdlr, Motor_t *wheels) {
	wheels[CHASSIS_WHEEL1_INDEX].tx_data = pid_single_loop_control(
			chassis_hdlr->mec_spd[CHASSIS_WHEEL1_INDEX],
		&(wheels[CHASSIS_WHEEL1_INDEX].motor_info.f_pid),
		wheels[CHASSIS_WHEEL1_INDEX].motor_feedback.rx_rpm,
		CHASSIS_TASK_EXEC_TIME*0.001);
	wheels[CHASSIS_WHEEL2_INDEX].tx_data = pid_single_loop_control(
			chassis_hdlr->mec_spd[CHASSIS_WHEEL2_INDEX],
		&(wheels[CHASSIS_WHEEL2_INDEX].motor_info.f_pid),
		wheels[CHASSIS_WHEEL2_INDEX].motor_feedback.rx_rpm,
		CHASSIS_TASK_EXEC_TIME*0.001);
	wheels[CHASSIS_WHEEL3_INDEX].tx_data = pid_single_loop_control(
			chassis_hdlr->mec_spd[CHASSIS_WHEEL3_INDEX],
		&(wheels[CHASSIS_WHEEL3_INDEX].motor_info.f_pid),
		wheels[CHASSIS_WHEEL3_INDEX].motor_feedback.rx_rpm,
		CHASSIS_TASK_EXEC_TIME*0.001);
	wheels[CHASSIS_WHEEL4_INDEX].tx_data = pid_single_loop_control(
			chassis_hdlr->mec_spd[CHASSIS_WHEEL4_INDEX],
		&(wheels[CHASSIS_WHEEL4_INDEX].motor_info.f_pid),
		wheels[CHASSIS_WHEEL4_INDEX].motor_feedback.rx_rpm,
		CHASSIS_TASK_EXEC_TIME*0.001);
}

/*
 * @brief 	  Inversely calculate the mecanum wheel speed
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void chassis_send_wheel_volts(Chassis_t *chassis_hdlr, Motor_t *wheels) {
	mecanum_wheel_calc_speed(chassis_hdlr);
	/* max +-16834 */
	for (int i = 0; i < 4; i++) {
		VAL_LIMIT(chassis_hdlr->mec_spd[i], -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
	}
	MotorSetMessage_t motor_set_message;
	motor_set_message.motor_can_volts[CHASSIS_WHEEL1_CAN_ID] = wheels[CHASSIS_WHEEL1_INDEX].tx_data;
	motor_set_message.motor_can_volts[CHASSIS_WHEEL2_CAN_ID] = wheels[CHASSIS_WHEEL2_INDEX].tx_data;
	motor_set_message.motor_can_volts[CHASSIS_WHEEL3_CAN_ID] = wheels[CHASSIS_WHEEL3_INDEX].tx_data;
	motor_set_message.motor_can_volts[CHASSIS_WHEEL4_CAN_ID] = wheels[CHASSIS_WHEEL4_INDEX].tx_data;
	motor_set_message.data_enable = (1 << CHASSIS_WHEEL1_CAN_ID) |
									(1 << CHASSIS_WHEEL2_CAN_ID) |
									(1 << CHASSIS_WHEEL3_CAN_ID) |
									(1 << CHASSIS_WHEEL4_CAN_ID);
	pub_message(MOTOR_SET, &motor_set_message);
}

/*
 * @brief 	  Update chassis gimbal axis data through rc
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void chassis_update_gimbal_coord(Chassis_t *chassis_hdlr, int16_t *channels) {
	/* controller data is not required to be filtered */
	chassis_hdlr->gimbal_axis.vx = channels[3]; // apply vx data here
	chassis_hdlr->gimbal_axis.vy = channels[2]; // apply vy data here
	chassis_hdlr->gimbal_axis.wz = channels[0]; // apply wz data here
//	else if(rc_hdlr->control_mode == PC_MODE){
//		/* x axis process */
//		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status == PRESSED)
//			/* why did you do this bro ? */
//			chassis_hdlr->gimbal_axis.vx = 0;
//		/* both not pressed, brake slowly*/
//		else if(rc_hdlr->pc.key.W.status != PRESSED && rc_hdlr->pc.key.S.status != PRESSED){
//			chassis_brake(&chassis_hdlr->gimbal_axis.vx, 1.0f, 2.0f);
//		}
//
//		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status != PRESSED){// check holding
//			chassis_hdlr->gimbal_axis.vx += CHASSIS_PC_RAMP_VALUE; // apply ramp-like mode to engage chassis
//			if(chassis_hdlr->gimbal_axis.vx >= chassis_hdlr->max_vx)
//				chassis_hdlr->gimbal_axis.vx = chassis_hdlr->max_vx;
//		}
//
//		if(rc_hdlr->pc.key.S.status == PRESSED && rc_hdlr->pc.key.W.status != PRESSED){// check holding
//			chassis_hdlr->gimbal_axis.vx -= CHASSIS_PC_RAMP_VALUE; // apply ramp-like mode to engage chassis
//			if(chassis_hdlr->gimbal_axis.vx < -chassis_hdlr->max_vx)
//				chassis_hdlr->gimbal_axis.vx = -chassis_hdlr->max_vx;
//		}
//
//		/* y axis process */
//		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status == PRESSED)
//			/* why did you do this bro ? */
//			chassis_hdlr->gimbal_axis.vy = 0;
//		/* both not pressed, brake slowly*/
//		else if(rc_hdlr->pc.key.A.status != PRESSED && rc_hdlr->pc.key.D.status != PRESSED){
//			chassis_brake(&chassis_hdlr->gimbal_axis.vy, 1.0f, 2.0f);
//		}
//
//		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status != PRESSED){// check holding
//			chassis_hdlr->gimbal_axis.vy -= CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
//			if(chassis_hdlr->gimbal_axis.vy < -chassis_hdlr->max_vy)
//				chassis_hdlr->gimbal_axis.vy = -chassis_hdlr->max_vy;
//		}
//
//		if(rc_hdlr->pc.key.D.status == PRESSED && rc_hdlr->pc.key.A.status != PRESSED){// check holding
//			chassis_hdlr->gimbal_axis.vy += CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
//			if(chassis_hdlr->gimbal_axis.vy > chassis_hdlr->max_vy)
//				chassis_hdlr->gimbal_axis.vy = chassis_hdlr->max_vy;
//		}
//
//		/* angular velocity process, only used in Gimbal Follow mode */
//		if(chassis_hdlr->chassis_act_mode == GIMBAL_FOLLOW){
//			if(rc_hdlr->pc.key.Q.status == PRESSED && rc_hdlr->pc.key.E.status == PRESSED)
//				/* why did you do this bro ? */
//				chassis_hdlr->gimbal_axis.wz = 0;
//			/* both not pressed, brake slowly*/
//			else if(rc_hdlr->pc.key.Q.status != PRESSED && rc_hdlr->pc.key.E.status != PRESSED){
//				chassis_brake(&chassis_hdlr->gimbal_axis.wz, 1.0f, 2.0f);
//			}
//
//			if(rc_hdlr->pc.key.Q.status == PRESSED && rc_hdlr->pc.key.E.status != PRESSED){// check holding
//				chassis_hdlr->gimbal_axis.wz -= CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
//				if(chassis_hdlr->gimbal_axis.wz < -chassis_hdlr->max_wz)
//					chassis_hdlr->gimbal_axis.wz = -chassis_hdlr->max_wz;
//			}
//
//			if(rc_hdlr->pc.key.E.status == PRESSED && rc_hdlr->pc.key.Q.status != PRESSED){// check holding
//				chassis_hdlr->gimbal_axis.wz += CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
//				if(chassis_hdlr->gimbal_axis.wz > chassis_hdlr->max_vy)
//					chassis_hdlr->gimbal_axis.wz = chassis_hdlr->max_vy;
//			}
//		}
//	}
}
/*
 * @brief 	  Update chassis ground data through rc
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void chassis_update_chassis_coord(Chassis_t *chassis_hdlr, int16_t *channels) {
	/*chassis coordinates only for debugging purpose, thus no pc control processing*/
	chassis_hdlr->vx = channels[3]; // apply vx data here
	chassis_hdlr->vy = channels[2]; // apply vy data here
	chassis_hdlr->wz = channels[0];
//	else if(rc_hdlr->control_mode == PC_MODE){
//		/* x axis process */
//		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status == PRESSED)
//			/* why did you do this bro ? */
//			chassis_hdlr->vx = 0;
//		/* both not pressed, brake slowly*/
//		else if(rc_hdlr->pc.key.W.status != PRESSED && rc_hdlr->pc.key.S.status != PRESSED){
//			chassis_brake(&chassis_hdlr->vx, 1.0f, 2.0f);
//		}
//
//		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status != PRESSED){// check holding
//			chassis_hdlr->vx += CHASSIS_PC_RAMP_VALUE ; // apply ramp-like mode to engage chassis
//			if(chassis_hdlr->vx >= chassis_hdlr->max_vx)
//				chassis_hdlr->vx = chassis_hdlr->max_vx;
//		}
//
//		if(rc_hdlr->pc.key.S.status == PRESSED && rc_hdlr->pc.key.W.status != PRESSED){// check holding
//			chassis_hdlr->vx -= CHASSIS_PC_RAMP_VALUE ; // apply ramp-like mode to engage chassis
//			if(chassis_hdlr->vx < -chassis_hdlr->max_vx)
//				chassis_hdlr->vx = -chassis_hdlr->max_vx;
//		}
//
//		/* y axis process */
//		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status == PRESSED)
//			/* why did you do this bro ? */
//			chassis_hdlr->vy = 0;
//		/* both not pressed, brake slowly*/
//		else if(rc_hdlr->pc.key.A.status != PRESSED && rc_hdlr->pc.key.D.status != PRESSED){
//			chassis_brake(&chassis_hdlr->vy, 1.0f, 2.0f);
//		}
//
//		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status != PRESSED){// check holding
//			chassis_hdlr->vy -= CHASSIS_PC_RAMP_VALUE ;// apply ramp-like mode to engage chassis
//			if(chassis_hdlr->vy < -chassis_hdlr->max_vy)
//				chassis_hdlr->vy = -chassis_hdlr->max_vy;
//		}
//
//		if(rc_hdlr->pc.key.D.status == PRESSED && rc_hdlr->pc.key.A.status != PRESSED){// check holding
//			chassis_hdlr->vy += CHASSIS_PC_RAMP_VALUE ;// apply ramp-like mode to engage chassis
//			if(chassis_hdlr->vy > chassis_hdlr->max_vy)
//				chassis_hdlr->vy = chassis_hdlr->max_vy;
//		}
//
//		chassis_hdlr->wz = 2.0*rc_hdlr->pc.mouse.x;
//		if(chassis_hdlr->wz < -chassis_hdlr->max_wz)
//			chassis_hdlr->wz = -chassis_hdlr->max_wz;
////		if(chassis_hdlr->wz > -chassis_hdlr->max_wz)
////			chassis_hdlr->wz = chassis_hdlr->max_wz;
//
//	}
}

/*
 * @brief Execute the chassis action mode:
 *			follow gimbal center| move_along gimbal coordinate |
 *			self-spinning while follow the gimbal coordinate | independent(ground coordinate)
 */
//FIXME: Didn't consider the acceleration. Acceleration can help us better explicit the buffer energy.
//		 But with more critical strict on power management.
void chassis_exec_act_mode(Chassis_t *chassis_hdlr){

	if(chassis_hdlr->chassis_mode == IDLE_MODE){
		chassis_hdlr->vx = 0;
		chassis_hdlr->vy = 0;
		chassis_hdlr->wz = 0;
	}
	else if(chassis_hdlr->chassis_act_mode == GIMBAL_CENTER){ // gyro mode
		/* The front of chassis always chases gimbal yaw's ecd center (aka Twist mode) */
		chassis_hdlr->vx = chassis_hdlr->gimbal_axis.vx;
		chassis_hdlr->vy = chassis_hdlr->gimbal_axis.vy;
		chassis_hdlr->wz = -pid_single_loop_control(0, &(chassis_hdlr->f_pid), chassis_hdlr->gimbal_yaw_rel_angle, CHASSIS_TASK_EXEC_TIME*0.001);
	}
	else if(chassis_hdlr->chassis_act_mode == GIMBAL_FOLLOW){ // encoder mode
		/* The chassis always move along gimbal's coord/axis , but not chasing yaw's center */
		chassis_hdlr->vx = chassis_hdlr->gimbal_axis.vx * arm_cos_f32(chassis_hdlr->gimbal_yaw_rel_angle) - chassis_hdlr->gimbal_axis.vy * arm_sin_f32(chassis_hdlr->gimbal_yaw_rel_angle);
		chassis_hdlr->vy = chassis_hdlr->gimbal_axis.vx * arm_sin_f32(chassis_hdlr->gimbal_yaw_rel_angle) + chassis_hdlr->gimbal_axis.vy * arm_cos_f32(chassis_hdlr->gimbal_yaw_rel_angle);
		chassis_hdlr->wz = 0;
//		if(rc.control_mode == CTRLER_MODE)
//			chassis_hdlr->wz = 0;
//		else if(rc.control_mode == PC_MODE)
//			chassis_hdlr->wz = chassis_hdlr->gimbal_axis.wz;
	}
	else if(chassis_hdlr->chassis_act_mode == SELF_GYRO){ // gyro or encoder mode
		/* The chassis always move along gimbal's coord/axis , meanwhile spinning the chassis with a fixed speed */
		chassis_hdlr->vx = chassis_hdlr->gimbal_axis.vx * arm_cos_f32(chassis_hdlr->gimbal_yaw_rel_angle) - chassis_hdlr->gimbal_axis.vy * arm_sin_f32(chassis_hdlr->gimbal_yaw_rel_angle);
		chassis_hdlr->vy = chassis_hdlr->gimbal_axis.vx * arm_sin_f32(chassis_hdlr->gimbal_yaw_rel_angle) + chassis_hdlr->gimbal_axis.vy * arm_cos_f32(chassis_hdlr->gimbal_yaw_rel_angle);
		/* for robots with slipring */
		//FIXME apply differential rotary control or use Q&E to change direction
		chassis_hdlr->wz =  CHASSIS_ECD_CONST_OMEGA * 3.5f;
	}
	else if(chassis_hdlr->chassis_act_mode == INDPET_MODE){ // encoder mode
		/* The chassis follow the ground axis
		 * Also can be used as sentry's chassis cmd
		 *  */
		chassis_hdlr->vx = chassis_hdlr->vx;
		chassis_hdlr->vy = chassis_hdlr->vy;
		chassis_hdlr->wz = chassis_hdlr->wz;//CHASSIS_SLEF_GYRO_ANG_VEL * 1.0f;
	}

	/* set limit axis speed */
#ifdef CHASSIS_POWER_LIMIT
//	uint8_t cur_robot_level = referee.robot_status_data.robot_level;
//	/* Chassis Power Management Starts Here */
//	if(cur_robot_level > 10 || cur_robot_level < 1){
//		cur_robot_level = chassis_hdlr->prev_robot_level;// Set prev level for secure
//	}
//	else if(cur_robot_level - cur_robot_level >= 5){ // This may indicate failure comm with ref
//		cur_robot_level = chassis_hdlr->prev_robot_level;
//	}
//	select_chassis_speed(chassis_hdlr, cur_robot_level);
#endif
#ifndef REF_CHASSIS_DEBUG
	VAL_LIMIT(chassis_hdlr->vx, -chassis_hdlr->max_vx, chassis_hdlr->max_vx);
	VAL_LIMIT(chassis_hdlr->vy, -chassis_hdlr->max_vy, chassis_hdlr->max_vy);
	if(chassis_hdlr->chassis_act_mode != GIMBAL_CENTER)
		// Gimbal center doesn't need to limit wz
		VAL_LIMIT(chassis_hdlr->wz, -chassis_hdlr->max_wz, chassis_hdlr->max_wz);
	else
		VAL_LIMIT(chassis_hdlr->wz, -1.5*chassis_hdlr->max_wz, 1.5*chassis_hdlr->max_wz);
#else
	VAL_LIMIT(chassis_hdlr->vx, -temp_max_vx, temp_max_vx);
	VAL_LIMIT(chassis_hdlr->vy, -temp_max_vy, temp_max_vy);
	VAL_LIMIT(chassis_hdlr->wz, -temp_max_wz, temp_max_wz);
#endif
	if(fabs(chassis_hdlr->wz) < 50.0f)
		/* PID dead zone risk management */
		chassis_hdlr->wz = 0;
}

/*
 * @brief brake the chassis slowly to avoid instant power overlimt
 */
void chassis_brake(float *vel, float ramp_step, float stop_threshold){
	if(*vel > 0)// both release -> brake
		*vel -= ramp_step;//brake need to be quicker
	else if(*vel < 0)
		*vel += ramp_step;
	if(fabs(*vel) < stop_threshold)
		*vel = 0;
}


void chassis_get_gimbal_rel_angles(Chassis_t *chassis_hdlr) {
	float rel_angles[2];
	BaseType_t new_rel_angle_message = peek_message(GIMBAL_REL_ANGLES, rel_angles, 0);
	if (new_rel_angle_message == pdTRUE) {
		chassis_hdlr->gimbal_yaw_rel_angle = rel_angles[0];
		chassis_hdlr->gimbal_pitch_rel_angle = rel_angles[1];
	}
}


void chassis_get_rc_info(Chassis_t *chassis_hdlr, int16_t *channels) {
	RCInfoMessage_t rc_info;
	BaseType_t new_message = peek_message(RC_INFO, &rc_info, 0);

	if (new_message == pdTRUE) {
		// TODO: Add input validation for modes and channels.
		BoardMode_t board_mode = rc_info.modes[0];
		BoardActMode_t act_mode = rc_info.modes[1];

		chassis_hdlr->chassis_mode = board_mode;
		chassis_hdlr->chassis_act_mode = act_mode;
		memcpy(channels, &(rc_info.channels), sizeof(int16_t) * 4);
	}
}


void chassis_get_wheel_feedback(Motor_t *wheels) {
	Motor_Feedback_t feedback[8];
	BaseType_t new_motor_read_message = peek_message(MOTOR_READ, feedback, 0);
	if (new_motor_read_message == pdTRUE) {
		memcpy(&(wheels[CHASSIS_WHEEL1_INDEX].motor_feedback), &(feedback[CHASSIS_WHEEL1_CAN_ID]), sizeof(Motor_Feedback_t));
		memcpy(&(wheels[CHASSIS_WHEEL2_INDEX].motor_feedback), &(feedback[CHASSIS_WHEEL2_CAN_ID]), sizeof(Motor_Feedback_t));
		memcpy(&(wheels[CHASSIS_WHEEL3_INDEX].motor_feedback), &(feedback[CHASSIS_WHEEL3_CAN_ID]), sizeof(Motor_Feedback_t));
		memcpy(&(wheels[CHASSIS_WHEEL4_INDEX].motor_feedback), &(feedback[CHASSIS_WHEEL4_CAN_ID]), sizeof(Motor_Feedback_t));
	}
}

/* define rc used count vars */
//int8_t chassis_pc_mode_toggle = 1; // encoder mode
//int8_t chassis_pc_submode_toggle = -1; // default to gimbal follow mode
//int32_t temp_toggle_count = 0;
/*
 * @brief     mode selection based on remote controller
 * @param[in] chassis: main chassis handler
 * @param[in] rc: main remote controller handler
 * */
//static void chassis_rc_mode_selection(Chassis_t* chassis_hdlr){

	/* Mode quick check:
	 * -----------------------------------------------------------------------
	 * | toggle flag | pc_mode_toggle | pc_submode_toggle | Mode			 |
	 * |             | 		(Ctrl)    |        (F)        |      			 |
	 * -----------------------------------------------------------------------
	 * |  		     |  	 1  	  |  		-1		  |	GIMBAL_FOLLOW    | __
	 * |			 |--------------------------------------------------------   | - Enconder mode
	 * |	         |  	 1   	  |  	    1         |	INDEPENDENT MODE | __|
	 * |	index	 |--------------------------------------------------------
	 * |	         |      -1        |  		-1        |	GIMBAL CENTER    | __
	 * |			 |--------------------------------------------------------   | - Gyro mode
	 * |	         |  	-1        |  	    1         |	SELF_GYRO	     | __|
	 * -----------------------------------------------------------------------
	 * */
	/* pc end mode selection */
//	else if(rc_hdlr->control_mode == PC_MODE) {
//		if(rc_hdlr->pc.key.key_buffer & KEY_BOARD_G){
//				/* if s1 down, then just shut down everything */
//				board_mode = IDLE_MODE;
//			}
//		else{
//			/* else just set up to patrol mode */
//			board_mode = PATROL_MODE;
//			/* update keys state */
////			if(rc_hdlr->pc.key.key_buffer & KEY_BOARD_CTRL)
//			if(rc_get_key_status(&rc_hdlr->pc.key.Ctrl) == RELEASED_TO_PRESS){ // check rising edge
//				chassis_pc_mode_toggle = -chassis_pc_mode_toggle;
//				temp_toggle_count++;
//			}
//			if(rc_get_key_status(&rc_hdlr->pc.key.F) == RELEASED_TO_PRESS) // check rising edge
//				chassis_pc_submode_toggle = -chassis_pc_submode_toggle;
//
//			/* mode decide */
//			if(chassis_pc_mode_toggle == -1 && chassis_pc_submode_toggle == -1){
//				/* chassis follow gimbal center while follow yaw axis */
//				act_mode = GIMBAL_CENTER;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//
//			else if(chassis_pc_mode_toggle == -1 && chassis_pc_submode_toggle == 1){
//				/* spinning chassis while follow yaw axis */
//				act_mode = SELF_GYRO;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//			else if(chassis_pc_mode_toggle == 1 && chassis_pc_submode_toggle == -1){
//				/* chassis only follow yaw axis */
//				act_mode = GIMBAL_FOLLOW;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//			else if(chassis_pc_mode_toggle == 1 && chassis_pc_submode_toggle == 1){
//				/* independent mode */
//				act_mode = INDPET_MODE;
//				/* update ground axis */
//				chassis_update_chassis_coord(chassis_hdlr, rc_hdlr);
//			}
//		}// else patrol mode
//	}//pc mode

	/* set modes */
//}

//void chassis_manual_gear_set(Chassis_t* chassis_hdlr, RemoteControl_t *rc_hdlr){
//	/* Manually set the levels of robot */
//	KeyStatus_t temp_upgrade_status = rc_get_key_status(&rc_hdlr->pc.key.V);
//	KeyStatus_t temp_downgrade_status = rc_get_key_status(&rc_hdlr->pc.key.Shift);
//	if( temp_upgrade_status == RELEASED_TO_PRESS && upgrade_pre_mode == RELEASED){
//		chassis_hdlr->cur_robot_level++;
//		upgrade_pre_mode = RELEASED_TO_PRESS;
//	}
//	else{
//		upgrade_pre_mode = temp_upgrade_status;
//	}
//	if(temp_downgrade_status == RELEASED_TO_PRESS && downgrade_pre_mode == RELEASED){
//		chassis_hdlr->cur_robot_level--;
//		downgrade_pre_mode = RELEASED_TO_PRESS;
//	}
//	else{
//		downgrade_pre_mode = temp_downgrade_status;
//	}
//
//	/* Safety check */
//	if(chassis_hdlr->cur_robot_level >= 10)
//		chassis_hdlr->cur_robot_level = 10;
//	if(chassis_hdlr->cur_robot_level <= 1)
//		chassis_hdlr->cur_robot_level = 1;
//
//	/* Set level */
//	switch(chassis_hdlr->cur_robot_level){
//			case 1: chassis_hdlr->max_vx = chassis_l1_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l1_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l1_hpf_spin_speed;break;
//			case 2: chassis_hdlr->max_vx = chassis_l2_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l2_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l2_hpf_spin_speed;break;
//			case 3: chassis_hdlr->max_vx = chassis_l3_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l3_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l3_hpf_spin_speed;break;
//			case 4: chassis_hdlr->max_vx = chassis_l4_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l4_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l4_hpf_spin_speed;break;
//			case 5: chassis_hdlr->max_vx = chassis_l5_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l5_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l5_hpf_spin_speed;break;
//			case 6: chassis_hdlr->max_vx = chassis_l6_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l6_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l6_hpf_spin_speed;break;
//			case 7: chassis_hdlr->max_vx = chassis_l7_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l7_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l7_hpf_spin_speed;break;
//			case 8: chassis_hdlr->max_vx = chassis_l8_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l8_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l8_hpf_spin_speed;break;
//			case 9: chassis_hdlr->max_vx = chassis_l9_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l9_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l9_hpf_spin_speed;break;
//			case 10: chassis_hdlr->max_vx = chassis_l10_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l10_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l10_hpf_spin_speed;break;
//	}
//}

void select_chassis_speed(Chassis_t* chassis_hdlr, uint8_t level){
	chassis_hdlr->prev_robot_level =  level;
	switch(level){
		case 1: chassis_hdlr->max_vx = chassis_l1_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l1_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l1_hpf_spin_speed;break;
		case 2: chassis_hdlr->max_vx = chassis_l2_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l2_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l2_hpf_spin_speed;break;
		case 3: chassis_hdlr->max_vx = chassis_l3_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l3_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l3_hpf_spin_speed;break;
		case 4: chassis_hdlr->max_vx = chassis_l4_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l4_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l4_hpf_spin_speed;break;
		case 5: chassis_hdlr->max_vx = chassis_l5_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l5_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l5_hpf_spin_speed;break;
		case 6: chassis_hdlr->max_vx = chassis_l6_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l6_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l6_hpf_spin_speed;break;
		case 7: chassis_hdlr->max_vx = chassis_l7_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l7_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l7_hpf_spin_speed;break;
		case 8: chassis_hdlr->max_vx = chassis_l8_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l8_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l8_hpf_spin_speed;break;
		case 9: chassis_hdlr->max_vx = chassis_l9_hpf_padding_speed;
				chassis_hdlr->max_vy = chassis_l9_hpf_padding_speed;
				chassis_hdlr->max_wz = chassis_l9_hpf_spin_speed;break;
		case 10: chassis_hdlr->max_vx = chassis_l10_hpf_padding_speed;
				 chassis_hdlr->max_vy = chassis_l10_hpf_padding_speed;
				 chassis_hdlr->max_wz = chassis_l10_hpf_spin_speed;break;
	}
}






#ifdef CHASSIS_POWER_LIMIT
/******************************************************************************************************************
 * CHASSIS POWER MANAGEMENT PROCESS WALKTHROUGH
 ******************************************************************************************************************
 * 	@attention: rules:
 *  The chassis power consumption of robots will be continuously monitored by the Referee System, and the robot
	chassis needs to run within the chassis power consumption limit. Considering it is difficult for a robot to control
	instantaneous output power when in motion, a buffer energy (Z) has been defined to avoid the consequent penalty.
 *  The buffer energy value of Hero, Standard and Sentry Robots is 60J.
 *  Excess Percentage: K = (Pr－Pl) / Pl * 100%, where Pr is the instantaneous Chassis Power Consumption output and
	Pl is the power consumption limit.
 *	W = Pt, meaning that if we running infantry with power=100w (the limit of the power=40w, level 1 infantry), after 1s
 	Hp would be deducted.(t = (100-40)/buffer_energy)

 	Power limit:(RMNA 2023 Rules manual)							Buffer energy:60J
 				Mode			Power-focused		HP-focues
 	infantry  level 1				 60                 45
 			  level 2			     80					50
 			  level 3   			100					55

 Strategy:
 * 1) Decrease the current value of each Mecanum wheel in equal proportions so that the total power does not exceed
 * 	  the power limit
 ******************************************************************************************************************/

/*
 * @brief     the power management of chassis strategy 1: proportionally decrease based on referee callback.
 * @param[in] chassis: main chassis handler
 * */
void chassis_power_limit_referee(Chassis_t* chassis_hdlr){
	int32_t total_current = 0;
	int32_t abs_current_weighted_sum = 0;
	//step 1： Read chassis power limit rx data from referee system (op: combine super capacity power buffer)
	get_chassis_ref_power_stat(chassis_hdlr, &referee);
	//step 2: Converts the power limit to the total maximum motor output current(threshold)

	//step 3: Compare the threshold with a DANGER value, determine the power limit ratio (possibly)
	/**************************************** Important FIXME **********************************************/
	//FIXME: We may need to consider using the current feedback to more precisely calc total current thus total
	//		 power(Pt = Sum(motor[i] for i=range(0:3)) * U(24v)), not simply hard-set - it may cause significant shift
	//		 if needed, we may also apply pid control for it.
	//FIXME: The condition here is too naive and not considering the sampling time
	//		 the critical condition should be like: W_buffer - W_danger = samplingTime * P_limit
	/**************************************** Important FIXME **********************************************/
	if(chassis_hdlr->ref_power_stat.power >= CHASSIS_POWER_THRESHOLD){ // FIXME: here we need to change the threshold
																		//based on the robots level feedback, refer to public defines
		// stop chassis immediately
		total_current = 0;
	}
	if(chassis_hdlr->ref_power_stat.power >= CHASSIS_POWER_DANGER){

		// decrease the chassis spd immediately
		total_current = CHASSIS_POWER_DANGER * POWER_TO_CURRENT; //random value, need to test
	}
	//step 4: Weighted assignment (soft-max filter) of the current motor output value (conditional judgment)
	//softmax(chassis_hdlr->mec_spd, 4); //softmax cannot handle negative value very well
	for(int i=0;i<CHASSIS_MAX_WHEELS;i++)
		abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
	for(int i=0;i<CHASSIS_MAX_WHEELS;i++)
		chassis_hdlr->mec_spd[i] = chassis_hdlr->mec_spd[i] * (abs(chassis_hdlr->mec_spd[i])/abs_current_weighted_sum);
}

/*
 * @brief     the power management of chassis strategy 2: proportionally decrease based on local motor data.
 * @param[in] chassis: main chassis handler
 * @param[in] local_power_limit
 *
 * @note: this function calculates the power based on the equation below:
 * 			P = UI, where the voltage U is 24V, and the current can be obtained from can feedbcak data.
 * */
void chassis_power_limit_local(Chassis_t* chassis_hdlr, uint16_t local_power_limit){
	/* step 1: get the current power */
	int16_t current_power = 0;
	int32_t total_current = 0;
	int32_t abs_current_weighted_sum = 0;

	for (int i=0; i<CHASSIS_MAX_WHEELS; i++){
		current_power += 24 * motor_data[i].motor_feedback.rx_current * CHASSIS_MAX_SPEED/20;//assume the feedbcak current has same range with tx data
	}

	/* check if the power over danger zone */
	if(current_power >= CHASSIS_POWER_THRESHOLD ){ //assume we don't have a power feedbcak, need to manually set limit
		total_current = 0;
	}
	else if(current_power >= local_power_limit){
		total_current = (int32_t)local_power_limit/24;
    }

	/*apply current */
	for(int i=0;i<CHASSIS_MAX_WHEELS;i++)
		abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
	for(int i=0;i<CHASSIS_MAX_WHEELS;i++)
		chassis_hdlr->mec_spd[i] = (int16_t)chassis_hdlr->mec_spd[i] * (abs(chassis_hdlr->mec_spd[i])/abs_current_weighted_sum);
}

/* get the latest current power and power limit from referee system */
void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t *ref){
	chassis_hdlr->ref_power_stat.current = ref->power_heat_data.chassis_current;
	chassis_hdlr->ref_power_stat.power   = ref->power_heat_data.chassis_power;
	chassis_hdlr->ref_power_stat.buffer_energy   = ref->power_heat_data.buffer_energy;
}
#endif

/* only for sentry begin */

#endif /*__CHASSIS_APP__*/
