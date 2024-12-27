/*******************************************************************************
* @file           : motor.c
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Aug, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __MOTOR_C__
#define __MOTOR_C__

#include <Gimbal_App.h>
#include <motor.h>
#include "usart.h"
#include "pid.h"
#include "math.h"
#include <string.h>
#include "public_defines.h"


static Motor motor_data[MOTOR_COUNT];


void init_motor_data(void) {
	for(int i=0;i<MOTOR_COUNT;i++){
		memset(&(motor_data[i].motor_feedback), 0, sizeof(Motor_Feedback_t));
	}
}

/**
  * @brief     Read feedback from the motor sensor
  * @param[in] can1/can2 type header
  * @retval    None
  */
void parse_motor_feedback(const uint8_t *can_data, Motor_Feedback_t *motor_feedback, uint8_t size) {
	for (int i = 0; i < size; i++){
		motor_feedback[i].rx_angle	 =(int16_t)(can_data[0] << 8 | can_data[1]);
		motor_feedback[i].rx_rpm	 =(int16_t)(can_data[2] << 8 | can_data[3]);
		motor_feedback[i].rx_current =(int16_t)(can_data[4] << 8 | can_data[5]);
		motor_feedback[i].rx_temp	 =(int16_t)(can_data[6]);
	}
}

/**
  * @brief     send can tx data toward canbus
  * @param[in] can1/can2 type header
  * @param[in] Stdid of can device
  * @param[in] data set to different can devices
  * @retval    None
  */
void set_motor_voltage(CAN_HandleTypeDef* hcan, int32_t id, int32_t d1, int32_t d2, int32_t d3, int32_t d4){
	CAN_TxHeaderTypeDef  tx_header;
	uint8_t				 tx_data[8];

	tx_header.StdId = id;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;

	tx_data[0] = d1 >> 8;
	tx_data[1] = d1;
	tx_data[2] = d2 >> 8;
	tx_data[3] = d2;
	tx_data[4] = d3 >> 8;
	tx_data[5] = d3;
	tx_data[6] = d4 >> 8;
	tx_data[7] = d4;

	HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, (uint32_t*)CAN_TX_MAILBOX0);
}

/**
  * @brief     initialize the motor parameters
  * @retval    None
  */
void motor_init(uint8_t motor_id, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf){
	pid_param_init(&(motor_data[motor_id].motor_info.f_pid), max_out_f, max_i_out_f, max_err_f, kp_f, ki_f, kd_f);
	pid_param_init(&(motor_data[motor_id].motor_info.s_pid), max_out_s, max_i_out_s, max_err_s, kp_s, ki_s, kd_s);
	ff_param_init(&(motor_data[motor_id].motor_info.ff), kf);
}



float calc_gimbal_motor_dual_pid(float ff_input, float f_cur_val, int motor_idx, float dt) {
	return pid_dual_loop_control(feedforward(&motor_data[motor_idx].motor_info.ff, ff_input),//pid+ff
								  &(motor_data[motor_idx].motor_info.f_pid),
								  &(motor_data[motor_idx].motor_info.s_pid),
								  f_cur_val,
								  motor_data[motor_idx].motor_feedback.rx_rpm,
								  dt);
}


float calc_gimbal_motor_single_pid(float target_value, int motor_idx, float dt) {
	return pid_single_loop_control(target_value,
									&(motor_data[motor_idx].motor_info.s_pid),
									motor_data[motor_idx].motor_feedback.rx_rpm,
									dt);
}


float calc_shoot_mag_dual_pid(float target_value, float f_cur_val, float s_cur_val, int motor_idx, float dt) {
	return pid_dual_loop_control(target_value,
								 &(motor_data[motor_idx].motor_info.f_pid),
								 &(motor_data[motor_idx].motor_info.s_pid),
								 f_cur_val,
								 s_cur_val,
								 dt);//pid without ff
}


/**
  * @brief     send can tx data toward canbus
  * @param[in] can1/can2 type header
  * @param[in] Stdid of can device
  * @param[in] velocity/angle set to different can devices
  * @retval    None
  */
//void set_motor_can_volt(int32_t *motor_tx_buffer, float a1, float a2, int32_t v3, int32_t v4, GimbalMotorMode_t mode, uint8_t idle_flag){
//	if(mode == ENCODE_MODE) {
//		if(idle_flag == 1){//Set idle mode
//			float new_yaw = pid_dual_loop_control(feedforward(&motor_data[yaw_id].motor_info.ff, a1),//pid+ff
//					  &(motor_data[yaw_id].motor_info.f_pid),
//					  &(motor_data[yaw_id].motor_info.s_pid),
//					  gimbal->yaw_cur_rel_angle,
//					  motor_data[yaw_id].motor_feedback.rx_rpm,
//					  GIMBAL_TASK_EXEC_TIME*0.001);
//			float new_pitch = pid_dual_loop_control(feedforward(&motor_data[pitch_id].motor_info.ff, a2),//pid+ff
//					  &(motor_data[pitch_id].motor_info.f_pid),
//					  &(motor_data[pitch_id].motor_info.s_pid),
//					  gimbal->pitch_cur_rel_angle,
//					  motor_data[pitch_id].motor_feedback.rx_rpm,
//					  GIMBAL_TASK_EXEC_TIME*0.001);
//
//			memcpy(&motor_tx_buffer[yaw_id], &new_yaw, sizeof(float));
//			memcpy(&motor_tx_buffer[pitch_id], &new_pitch, sizeof(float));
//		} else {
//			float new_yaw = pid_dual_loop_control(feedforward(&motor_data[yaw_id].motor_info.ff, a1),//pid+ff
//					  &(motor_data[yaw_id].motor_info.f_pid),
//					  &(motor_data[yaw_id].motor_info.s_pid),
//					  gimbal->yaw_total_rel_angle,
//					  motor_data[yaw_id].motor_feedback.rx_rpm,
//					  GIMBAL_TASK_EXEC_TIME*0.001);
//			float new_pitch = pid_dual_loop_control(feedforward(&motor_data[pitch_id].motor_info.ff, a2),//pid+ff
//					  &(motor_data[pitch_id].motor_info.f_pid),
//					  &(motor_data[pitch_id].motor_info.s_pid),
//					  gimbal->pitch_cur_rel_angle,
//					  motor_data[pitch_id].motor_feedback.rx_rpm,
//					  GIMBAL_TASK_EXEC_TIME*0.001);
//
//			memcpy(&motor_tx_buffer[yaw_id], &new_yaw, sizeof(float));
//			memcpy(&motor_tx_buffer[pitch_id], &new_pitch, sizeof(float));
//		}
//	} else if(mode == GYRO_MODE) {
//
//		float new_yaw = pid_dual_loop_control(feedforward(&motor_data[yaw_id].motor_info.ff, a1),//pid+ff
//				  &(motor_data[yaw_id].motor_info.f_pid),
//				  &(motor_data[yaw_id].motor_info.s_pid),
//				  gimbal->final_abs_yaw,
//				  motor_data[yaw_id].motor_feedback.rx_rpm,
//				  GIMBAL_TASK_EXEC_TIME*0.001);//pid+ff
//		float new_pitch = pid_dual_loop_control(feedforward(&motor_data[pitch_id].motor_info.ff, a2),//pid+ff, pitch always use rel angle from encoder
//				  &(motor_data[pitch_id].motor_info.f_pid),
//				  &(motor_data[pitch_id].motor_info.s_pid),
//					  gimbal->pitch_cur_rel_angle,
////														  gimbal.pitch_cur_abs_angle,
//					  motor_data[pitch_id].motor_feedback.rx_rpm,
//					  GIMBAL_TASK_EXEC_TIME*0.001);//pid+ff
//
//		memcpy(&motor_tx_buffer[yaw_id], &new_yaw, sizeof(float));
//		memcpy(&motor_tx_buffer[pitch_id], &new_pitch, sizeof(float));
//	}
////	} else if(control_indicator == SINGLE_LOOP_PID_CONTROL) {
////		// only for spd control, dual loop control in the shoot app
////		float new_mag_2006 = pid_single_loop_control(v3,
////				&(motor_data[mag_2006_id].motor_info.s_pid),
////			      motor_data[mag_2006_id].motor_feedback.rx_rpm,
////				  SHOOT_TASK_EXEC_TIME*0.001);
////
////		memcpy(&motor_tx_buffer[mag_2006_id], &new_mag_2006, sizeof(float));
////	}
//}



void set_motor_can_current(int32_t v1, int32_t v2, int32_t v3, int32_t v4, int32_t control_indicator){
	if(control_indicator == DUAL_LOOP_PID_CONTROL){// only for hero magazine
		/* implemented in shoot app */
	}
	else if(control_indicator == SINGLE_LOOP_SHOOT_CONTROL){
		motor_data[fric_left_id].tx_data = pid_single_loop_control(v1,
																&(motor_data[fric_left_id].motor_info.f_pid),
															    motor_data[fric_left_id].motor_feedback.rx_current,
																SHOOT_TASK_EXEC_TIME*0.001);
		motor_data[fric_right_id].tx_data = pid_single_loop_control(v2,
																&(motor_data[fric_right_id].motor_info.f_pid),
																motor_data[fric_right_id].motor_feedback.rx_current,
																SHOOT_TASK_EXEC_TIME*0.001);
	}
	else{
		motor_data[wheel_id1].tx_data = pid_single_loop_control(v1,
														&(motor_data[wheel_id1].motor_info.f_pid),
													    motor_data[wheel_id1].motor_feedback.rx_rpm,
														CHASSIS_TASK_EXEC_TIME*0.001);
		motor_data[wheel_id2].tx_data = pid_single_loop_control(v2,
														&(motor_data[wheel_id2].motor_info.f_pid),
													    motor_data[wheel_id2].motor_feedback.rx_rpm,
														CHASSIS_TASK_EXEC_TIME*0.001);
		motor_data[wheel_id3].tx_data = pid_single_loop_control(v3,
														&(motor_data[wheel_id3].motor_info.f_pid),
													    motor_data[wheel_id3].motor_feedback.rx_rpm,
														CHASSIS_TASK_EXEC_TIME*0.001);
		motor_data[wheel_id4].tx_data = pid_single_loop_control(v4,
														&(motor_data[wheel_id4].motor_info.f_pid),
													    motor_data[wheel_id4].motor_feedback.rx_rpm,
														CHASSIS_TASK_EXEC_TIME*0.001);
	}
}


#ifdef GIMBAL_MOTOR_DEBUG
/**
  * @brief     initialize the motor parameters when debugging teh gimbal motor
  * @retval    None
  */
void motor_debug_init(uint8_t motor_id, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf){
	pid_param_init(&tune_pid_f, max_out_f, max_i_out_f, max_err_f, kp_f, ki_f, kd_f);
	pid_param_init(&tune_pid_s, max_out_s, max_i_out_s, max_err_s, kp_s, ki_s, kd_s);
	ff_param_init(&(motor_data[motor_id].motor_info.ff), kf);

}

/**
  * @brief     send can tx data toward canbus
  * @param[in] can1/can2 type header
  * @param[in] Stdid of can device
  * @param[in] velocity/angle set to different can devices
  * @retval    None
  */
void set_motor_debug_can_volt(float a1, float a2, int32_t v3, int32_t v4, int32_t control_indicator, GimbalMotorMode_t mode){

	if(control_indicator == DUAL_LOOP_PID_CONTROL && mode == ENCODE_MODE){
			motor_data[yaw_id].tx_data = pid_dual_loop_control(feedforward(&motor_data[yaw_id].motor_info.ff, a1),//pid+ff
														  &(tune_pid_f),
														  &(tune_pid_s),
														  in_out_map(gimbal_get_ecd_rel_angle(motor_data[yaw_id].motor_feedback.rx_angle, YAW_ECD_CENTER),
														  			 -4095,4095,-PI,PI),
														  motor_data[yaw_id].motor_feedback.rx_rpm);
			motor_data[pitch_id].tx_data = pid_dual_loop_control(feedforward(&motor_data[pitch_id].motor_info.ff, a2),//pid+ff
														  &(tune_pid_f),
														  &(tune_pid_s),
                                    					  in_out_map(gimbal_get_ecd_rel_angle(motor_data[pitch_id].motor_feedback.rx_angle, PITCH_ECD_CENTER),
																     -4095,4095,-PI,PI),
														  motor_data[pitch_id].motor_feedback.rx_rpm);
		}
	else if(control_indicator == DUAL_LOOP_PID_CONTROL && mode == GYRO_MODE){
			motor_data[yaw_id].tx_data = pid_dual_loop_control(feedforward(&motor_data[yaw_id].motor_info.ff, a1),//pid+ff
														  &(tune_pid_f),
														  &(tune_pid_f),
														  gimbal.yaw_cur_abs_angle,
														  motor_data[yaw_id].motor_feedback.rx_rpm) + feedforward(&motor_data[yaw_id].motor_info.ff, a1);//pid+ff
			motor_data[pitch_id].tx_data = pid_dual_loop_control(feedforward(&motor_data[pitch_id].motor_info.ff, a2),//pid+ff, pitch always use rel angle from encoder
														  &(tune_pid_f),
														  &(tune_pid_f),
														  in_out_map(gimbal_get_ecd_rel_angle(motor_data[pitch_id].motor_feedback.rx_angle, PITCH_ECD_CENTER),
														  		     -4095,4095,-PI,PI),
														  motor_data[pitch_id].motor_feedback.rx_rpm)+ feedforward(&motor_data[pitch_id].motor_info.ff, a2);//pid+ff


	}
	else if(control_indicator == SINGLE_LOOP_PID_CONTROL){
			// only for spd control, dual loop control in the shoot app
			motor_data[mag_2006_id].tx_data = pid_single_loop_control(v3,
															&(motor_data[mag_2006_id].motor_info.s_pid),
														      motor_data[mag_2006_id].motor_feedback.rx_rpm);
		}
}
#endif

#endif /* __MOTOR_C__ */
