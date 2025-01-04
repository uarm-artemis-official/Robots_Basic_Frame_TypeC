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

#include <motor.h>


void motor_data_init(Motor_t *motor) {
	memset(&(motor->motor_feedback), 0, sizeof(Motor_Feedback_t));
}

/**
  * @brief     initialize the motor parameters
  * @retval    None
  */
void motor_init(Motor_t *motor, int32_t max_out_f, float max_i_out_f, float max_err_f, float kp_f, float ki_f, float kd_f,
								  int32_t max_out_s, float max_i_out_s, float max_err_s, float kp_s, float ki_s, float kd_s,
								  float kf) {
	pid_param_init(&(motor->motor_info.f_pid), max_out_f, max_i_out_f, max_err_f, kp_f, ki_f, kd_f);
	pid_param_init(&(motor->motor_info.s_pid), max_out_s, max_i_out_s, max_err_s, kp_s, ki_s, kd_s);
	ff_param_init(&(motor->motor_info.ff), kf);
}

/**
  * @brief     Read feedback from the motor sensor
  * @param[in] can1/can2 type header
  * @retval    None
  */
void parse_motor_feedback(const uint8_t *can_data, Motor_Feedback_t *motor_feedback) {
	motor_feedback->rx_angle   = (((int16_t) can_data[0]) << 8) | ((int16_t) can_data[1]);
	motor_feedback->rx_rpm	   = (((int16_t) can_data[2]) << 8) | ((int16_t) can_data[3]);
	motor_feedback->rx_current = (((int16_t) can_data[4]) << 8) | ((int16_t) can_data[5]);
	motor_feedback->rx_temp	   = (int16_t) (can_data[6]);
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

#endif /* __MOTOR_C__ */
