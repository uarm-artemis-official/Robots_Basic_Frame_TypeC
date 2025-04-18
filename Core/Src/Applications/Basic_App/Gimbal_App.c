/*******************************************************************************
* @file           : Gimbal_App.c
* @brief          : gimbal task
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __GIMBAL_APP_C__
#define __GIMBAL_APP_C__

#include "Gimbal_App.h"


static Gimbal_t gimbal;
static int16_t gimbal_channels[2];

static Motor_t gimbal_motors[2];
//static int32_t motor_tx_buffer[8];

/* With encoder mode, task execution time (per loop): 1ms */
void Gimbal_Task_Function(void const * argument)
{
	/* gimbal task LD indicator */
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);

	/* init gimbal task */
	gimbal_task_init(&gimbal, gimbal_motors);

	/* reset calibration using ramp function */
	gimbal_calibration_reset(&gimbal, gimbal_motors);

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(GIMBAL_TASK_EXEC_TIME);
	xLastWakeTime = xTaskGetTickCount();

	/**
	 * Assumptions before entering loop:
	 *  - IMU is active and posting attitude to IMU_READINGS topic.
	 *  - Gimbal is calibrated and centered at relative 0.
	 *  - Gimbal IMU centers for yaw and pitch are set.
	 */
	for(;;){
		gimbal_get_rc_info(&gimbal);
		gimbal_get_motor_feedback(&gimbal, gimbal_motors);
		gimbal_get_imu_headings(&gimbal);

		gimbal_safe_mode_switch(&gimbal);
		gimbal_update_headings(&gimbal, gimbal_motors);
		gimbal_update_targets(&gimbal, gimbal_channels);

		/* Update previous mode */
		gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;

		gimbal_cmd_exec(&gimbal, gimbal_motors);

		gimbal_send_motor_volts(gimbal_motors);
		gimbal_send_rel_angles(&gimbal);

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/*
 * @brief     set the gimbal board work mode:
 * 				patrol | detected armor | Auto_Poilt | IDLE(no action) | Debug(remote control)
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: Board work mode
 * */
void gimbal_set_board_mode(Gimbal_t *gbal, BoardMode_t mode){
	gbal->gimbal_mode = mode;
}
/*
 * @brief     determime the mode for gimbal actions:
 * 				follow gimbal (master) | follow chassis (slave) or independent
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: act mode
 * */
void gimbal_set_act_mode(Gimbal_t *gbal, BoardActMode_t mode){
	gbal->gimbal_act_mode = mode;
	gbal->prev_gimbal_act_mode = mode;
}

/*
 * @brief 	  set motor mode: gyro | encoder
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: motor mode
 * */
void gimbal_set_motor_mode(Gimbal_t *gbal, GimbalMotorMode_t mode){
	gbal->prev_gimbal_motor_mode = gbal->gimbal_motor_mode;
	gbal->gimbal_motor_mode = mode;
}

/*
 * @brief     Initilize Gimbal task,
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_task_init(Gimbal_t *gbal, Motor_t *g_motors) {
	/* reset gimbal data */
	gimbal_reset_data(gbal, g_motors);

	/* init motor pid */
	// angular pid based on radian(-pi, pi), speed pid based on rpm(-15000, 15000)
	motor_init(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), max_out_angle_yaw,  max_I_out_angle_yaw, max_err_angle_yaw, kp_angle_yaw, ki_angle_yaw, kd_angle_yaw,
					   max_out_spd_yaw, max_I_out_spd_yaw, max_err_spd_yaw, kp_spd_yaw, ki_spd_yaw, kd_spd_yaw,
					   kf_spd_yaw);//spd ff gain

	motor_init(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), max_out_angle_pitch,  max_I_out_angle_pitch, max_err_angle_pitch, kp_angle_pitch, ki_angle_pitch, kd_angle_pitch,
					     max_out_spd_pitch, max_I_out_spd_pitch, max_err_spd_pitch, kp_spd_pitch, ki_spd_pitch, kd_spd_pitch,
					     kf_spd_pitch);//spd ff gain

	/* set init gimbal mode */
	gimbal_set_board_mode(gbal, PATROL_MODE);
	gimbal_set_act_mode(gbal, INDPET_MODE);
	gimbal_set_motor_mode(gbal, ENCODE_MODE);

//	memset(motor_tx_buffer, 999999, 32);
}

/*
 * @brief     the initialization process of the gimbal task,
 * 			  centering and ranging the motors using ecd
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_calibration_reset(Gimbal_t *gbal, Motor_t *g_motors) {
	/* init ramp functions*/
	ramp_init(&(gbal->yaw_ramp), 1500);//1.5s init
	ramp_init(&(gbal->pitch_ramp), 1500);

	// Center gimbal
	gbal->yaw_target_angle = 0;
	gbal->pitch_target_angle = 0;
	for(;;) {
		gimbal_get_motor_feedback(gbal, g_motors);
		gimbal_update_headings(gbal, g_motors);

		 if (fabs(gbal->yaw_rel_angle) < (2.0f * DEGREE2RAD)) {
			 break;
		 }

//		float yaw_diff = gimbal_calc_ecd_rel_angle(gbal->yaw_ecd_center, g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_angle);
//		float pitch_diff = gimbal_calc_ecd_rel_angle(gbal->pitch_ecd_center, g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_angle);
		float yaw_diff = calc_rel_angle(gbal->yaw_target_angle, gbal->yaw_rel_angle);
		float pitch_diff = calc_rel_angle(gbal->pitch_target_angle, gbal->pitch_rel_angle);

		g_motors[GIMBAL_YAW_MOTOR_INDEX].tx_data = pid2_dual_loop_control(&(gbal->yaw_f_pid),
					&(gbal->yaw_s_pid),
					0,
					yaw_diff,
					g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_rpm,
					GIMBAL_TASK_EXEC_TIME * 0.001,
					GIMBAL_TASK_EXEC_TIME * 0.001);

		g_motors[GIMBAL_PITCH_MOTOR_INDEX].tx_data = pid2_dual_loop_control(&(gbal->pitch_f_pid),
				&(gbal->pitch_s_pid),
				0,
				pitch_diff,
				g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_rpm,
				GIMBAL_TASK_EXEC_TIME * 0.001,
				GIMBAL_TASK_EXEC_TIME * 0.001);

		gimbal_send_motor_volts(g_motors);
		vTaskDelay(GIMBAL_TASK_EXEC_TIME);
	}

//	const int num_samples = 10;
//	int count = 0;
//	float avg_headings[2];
//	for (;;) {
//		uint8_t imu_ready_status = 0;
//		BaseType_t imu_ready = peek_message(IMU_READY, &imu_ready_status, 0);
//		if (imu_ready == pdTRUE && imu_ready_status == 1) {
//			if (count == num_samples) break;
//
//			float headings[2];
//			BaseType_t imu_posting = peek_message(IMU_READINGS, headings, 0);
//			if (imu_posting == pdTRUE) {
//				gimbal_update_imu_angle(gbal, headings[0], headings[1]);
//				avg_headings[0] += gbal->yaw_imu_angle;
//				avg_headings[1] += gbal->pitch_imu_angle;
//				count++;
//			}
//		}
//		osDelay(20);
//	}
//	avg_headings[0] /= num_samples;
//	avg_headings[1] /= num_samples;
//
//	gbal->yaw_imu_center = avg_headings[0];
	// TODO: Uncomment after implementing pitch centering.
//			gbal->pitch_imu_center = headings[1];
}

/*
 * @brief     Reset all data internal gimbal struct
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_reset_data(Gimbal_t *gbal, Motor_t *g_motors) {
	motor_data_init(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]));
	motor_data_init(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]));

	// Initialize non-zero Gimbal_t fields.
	memset(gbal, 0, sizeof(Gimbal_t));
	gbal->yaw_ecd_center = YAW_ECD_CENTER; // center position of the yaw motor - encoder
	gbal->pitch_ecd_center = PITCH_ECD_CENTER;
	gbal->euler_angle.timestamp = dwt_getCnt_us();

	pid2_init(&(gbal->yaw_f_pid), kp_angle_yaw, ki_angle_yaw, kd_angle_yaw, beta_angle_yaw, yeta_angle_yaw, -max_out_angle_yaw, max_out_angle_yaw);
	pid2_init(&(gbal->yaw_s_pid), kp_spd_yaw, ki_spd_yaw, kd_spd_yaw, beta_spd_yaw, yeta_spd_yaw, -max_out_spd_yaw, max_out_spd_yaw);
	pid2_init(&(gbal->pitch_f_pid), kp_angle_pitch, ki_angle_pitch, kd_angle_pitch, beta_angle_pitch, yeta_angle_pitch, -max_out_angle_pitch, max_out_angle_pitch);
	pid2_init(&(gbal->pitch_s_pid), kp_spd_pitch, ki_spd_pitch, kd_spd_pitch, beta_spd_pitch, yeta_spd_pitch, -max_out_spd_pitch, max_out_spd_pitch);

	init_folp_filter(&gbal->folp_f_yaw, 0.90f);
	init_folp_filter(&gbal->folp_f_pitch, 0.99f);

	init_ewma_filter(&gbal->ewma_f_x, 0.8f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_y, 0.8f);//0.6 for older client
	init_ewma_filter(&gbal->ewma_f_aim_yaw, 0.95f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_aim_pitch, 0.95f);//0.6 for older client

//	init_swm_filter(&gbal->swm_f_x, 20);// window size 50
//	init_swm_filter(&gbal->swm_f_y, 20);

	memset(&(gbal->ahrs_sensor), 0, sizeof(AhrsSensor_t));
	memset(&(gbal->euler_angle), 0, sizeof(Attitude_t));

	kalmanCreate(&(gbal->kalman_f), 0.001, 0.01);//0.0005 0.02
	gbal->prev_gimbal_motor_mode = ENCODE_MODE;
}

/******************  MODE SELECTION FUNCTIONS BELOW ********************/
//void gimbal_get_raw_mpu_data(Gimbal_t *gbal, IMU_t *imu_hldr){
//	memcpy(&(gbal->ahrs_sensor), &(imu_hldr->ahrs_sensor), sizeof(AhrsSensor_t));
//}
//
///*
// * @brief     Copy the gyroscope data from imu and calculate quaternion
// * 			  and euler's angle through attitude-breakdown algorithms.
// * @param[in] gimbal: main gimbal handler
// * */
//void gimbal_get_euler_angle(Gimbal_t *gbal){
//	gimbal_get_raw_mpu_data(gbal, &imu); // copy data to avoid mem leaks
////	atti_math_calc(&gbal->ahrs_sensor, &gbal->euler_angle); //complementary filter parsed angle
////	mahony_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));	//mahony algo
//	madgwick_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));  //madgwick algo
//}

/*
 * @brief     Copy the gyroscope data from imu and calculate quaternion
 * 			  and euler's absolute angle through attitude-breakdown algorithms.
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_update_imu_angle(Gimbal_t *gbal, float yaw, float pitch) {

	 /* get timestamp */
	 uint32_t DWTcnt = dwt_getCnt_us();// systemclock_core 168MHz ->usec
//	 uint32_t delta_t = min(DWTcnt - gbal->euler_angle.timestamp, 3000);

	 /* filter the yaw angle data to handle shift */
	 gbal->euler_angle.yaw = first_order_low_pass_filter(&(gbal->folp_f_yaw), yaw);
	 gbal->euler_angle.pitch = first_order_low_pass_filter(&(gbal->folp_f_pitch), pitch);
	 gbal->euler_angle.timestamp = DWTcnt;
}
/*
 * @brief     update the relevant encoder angle
 * @param[in] gbal: main gimbal handler
 * */
//void gimbal_get_ecd_fb_data(Gimbal_t *gbal) {
//	Motor_Feedback_t motor_feedback_buffer[MOTOR_COUNT];
//	BaseType_t new_feedback_message = peek_message(MOTOR_READ, motor_feedback_buffer, 0);
//	if (new_feedback_message == pdTRUE) {
//		memcpy(&(gbal->yaw_ecd_fb), motor_feedback_buffer + GIMBAL_YAW_CAN_ID * 8, sizeof(Motor_Feedback_t));
//		gbal->yaw_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->yaw_ecd_fb.rx_angle, gbal->yaw_ecd_center);
//		memcpy(&(gbal->pitch_ecd_fb), motor_feedback_buffer + GIMBAL_PITCH_CAN_ID * 8, sizeof(Motor_Feedback_t));
//		gbal->pitch_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->pitch_ecd_fb.rx_angle, gbal->pitch_ecd_center);
//		gimbal_update_ecd_rel_angle(gbal);
//	}
//}

/**
 * Calculates the smallest relative angle of angle2 relative to angle1.
 *
 * Assumes angle1 and angle2 are both in radians ([0, 2 * PI] or (-PI, PI).
 * angle1 and angle2 must have the same origin!!!
 *
 * The resulting angle assumes clockwise is negative and counter-clockwise is positive.
 */
float calc_rel_angle(float angle1, float angle2) {
	float cw_magnitude = angle1 - angle2;
	if (cw_magnitude < 0) cw_magnitude += 2 * PI;

	float ccw_magnitude = angle2 - angle1;
	if (ccw_magnitude < 0) ccw_magnitude += 2 * PI;

	if (cw_magnitude < ccw_magnitude) {
		return -cw_magnitude;
	} else {
		return ccw_magnitude;
	}
}


/*
 * @brief     Get relative angle of gimbal motors.
 * @param[in] raw_ecd: abs yaw ecd angle from feedback
 * @param[in] center_offset: the center offset of ecd mode
 * */
int16_t gimbal_calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset)
{
  /* declare a 16-bit signed integer tmp to store the relative angle */
  int16_t tmp = 0;

  /*  check if the center offset is in the upper half of the ecd range (4096-8191) */
  if (center_offset >= 4096){
    /*  check if the raw ecd value is in the same half circle as the center offset */
    if (raw_ecd > center_offset - 4096)
      /*  the raw ecd value is in the same half circle as the center offset
          so, simply subtract the center offset from the raw ecd to get the relative angle */
      tmp = raw_ecd - center_offset;
    else
      /*  the raw ecd value is in the different half circle from the center offset
          subtract the center offset from the raw ecd plus 8192 to get the relative angle */
      tmp = raw_ecd + 8192 - center_offset;
  }
  /*  check if the center offset is in the lower half of the ecd range (0-4095) */
  else{
    /*  check if the raw ecd value is in the different half circle from the center offset */
    if (raw_ecd > center_offset + 4096)
      /*  the raw ecd value is in the different half circle from the center offset
          subtract the center offset and 8192 from the raw ecd to get the relative angle */
      tmp = raw_ecd - 8192 - center_offset;
    else
      /*  the raw ecd value is in the same half circle as the center offset
          so, simply subtract the center offset from the raw ecd to get the relative angle */
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}
/*
 * @brief     Update gimbal motor relative and mapped angle using encoder
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_update_ecd_angles(Gimbal_t *gbal, Motor_t *g_motors) {
	int16_t yaw_ecd_rel_angle = gimbal_calc_ecd_rel_angle(g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_angle, gbal->yaw_ecd_center);
	int16_t pitch_ecd_rel_angle = gimbal_calc_ecd_rel_angle(g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_angle, gbal->pitch_ecd_center);

	gbal->yaw_ecd_angle = in_out_map(yaw_ecd_rel_angle,-4095,4096,-PI,PI);
	gbal->pitch_ecd_angle = in_out_map(pitch_ecd_rel_angle,-4095,4096,-PI,PI);
}

/*
 * @brief     Ensure the mode switch safely
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_safe_mode_switch(Gimbal_t *gbal){
 if(gbal->prev_gimbal_motor_mode != gbal->gimbal_motor_mode) {
	 // TODO: Implement synchronization method if switching from encoder -> gyro results
	 //       in noticeable shifts due to changes in imu_center due to gyroscopic drift.
   }
}


void gimbal_calc_channels_to_angles(const int16_t g_channels[2], float deltas[2]) {
	/**
	 * deltas[0] - delta yaw
	 * deltas[1] - delta pitch
	 */
	deltas[0] = in_out_map((float) g_channels[0],
			-CHANNEL_OFFSET_MAX_ABS_VAL,
			CHANNEL_OFFSET_MAX_ABS_VAL,
			-2.0f * DEGREE2RAD, 2.0f * DEGREE2RAD);
	deltas[1] = in_out_map((float) g_channels[1],
			-CHANNEL_OFFSET_MAX_ABS_VAL,
			CHANNEL_OFFSET_MAX_ABS_VAL,
			-1.0f * DEGREE2RAD, 1.0f * DEGREE2RAD);
}

/******************  MODE SELECTION FUNCTIONS ABOVE ********************/

/*
 * @brief     set the target angle with limited range
 * @param[in] gbal: main gimbal handler
 * @param[in] target yaw and pitch relative angle(-pi, pi)
 */
void gimbal_set_limited_angle(Gimbal_t *gbal, float new_yaw_target_angle, float new_pitch_target_angle){
	gbal->yaw_target_angle = new_yaw_target_angle;
	gbal->pitch_target_angle = new_pitch_target_angle;
	/* only set limit for yaw where is no slipring */
//	VAL_LIMIT(gbal->yaw_tar_angle,
//				   -PI,
//				    PI);
	/* set the limit for pitch */
	VAL_LIMIT(gbal->pitch_target_angle,
				   -PITCH_GYRO_DELTA,
					PITCH_GYRO_DELTA);
}


/*
 * @brief     mode selection based on remote controller
 * @param[in] chassis: main chassis handler
 * @param[in] rc: main remote controller handler
 * */
void gimbal_set_modes(Gimbal_t* gbal, uint8_t modes[3]) {
	BoardMode_t board_mode = modes[0];
	BoardActMode_t act_mode = modes[1];
	gimbal_set_board_mode(gbal, board_mode);
	gimbal_set_act_mode(gbal, act_mode);

	if (gbal->gimbal_act_mode == SELF_GYRO || gbal->gimbal_act_mode == GIMBAL_FOLLOW) {
		gimbal_set_motor_mode(gbal, GYRO_MODE);
	} else {
		gimbal_set_motor_mode(gbal, ENCODE_MODE);
	}
}


void gimbal_get_rc_info(Gimbal_t *gbal) {
#ifndef ENABLE_MANUAL_MODE_SET
	RCInfoMessage_t rc_info;
	BaseType_t new_rc_info_message = peek_message(RC_INFO, &rc_info, 0);
	if (new_rc_info_message == pdTRUE) {
		gimbal_set_modes(gbal, rc_info.modes);
		memcpy(gimbal_channels, rc_info.channels, sizeof(int16_t) * 2);
	}
#else
	gimbal_set_board_mode(gbal, AUTO_AIM_MODE);
	gimbal_set_act_mode(gbal, GIMBAL_FOLLOW);
	gimbal_set_motor_mode(gbal, ENCODE_MODE);
#endif
}


void gimbal_get_motor_feedback(Gimbal_t *gbal, Motor_t *g_motors) {
	Motor_Feedback_t motor_feedback_buffer[MOTOR_COUNT];
	BaseType_t new_feedback_message = peek_message(MOTOR_READ, motor_feedback_buffer, 0);
	if (new_feedback_message == pdTRUE) {
		memcpy(&(g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback), &(motor_feedback_buffer[GIMBAL_YAW_CAN_ID]), sizeof(Motor_Feedback_t));
		memcpy(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback), &(motor_feedback_buffer[GIMBAL_PITCH_CAN_ID]), sizeof(Motor_Feedback_t));
	}
}


void gimbal_get_imu_headings(Gimbal_t *gbal) {
	float imu_readings[2];
	BaseType_t new_imu_message = peek_message(IMU_READINGS, imu_readings, 0);

	if (new_imu_message == pdTRUE) {
		gimbal_update_imu_angle(gbal, imu_readings[0], imu_readings[1]);
	} else {
		// TODO: Implement error handling
	}
}


void gimbal_send_rel_angles(Gimbal_t *gbal) {
	CANCommMessage_t rel_angle_message;
	rel_angle_message.topic_name = GIMBAL_REL_ANGLES;

	float temp_yaw = -gbal->yaw_rel_angle;
	memcpy(rel_angle_message.data, &temp_yaw, sizeof(float));
	memcpy(&(rel_angle_message.data[4]), &(gbal->pitch_rel_angle), sizeof(float));

	pub_message(COMM_OUT, &rel_angle_message);
}


void gimbal_update_headings(Gimbal_t *gbal, Motor_t *g_motors) {
	if (gbal->gimbal_motor_mode == GYRO_MODE) {
		gbal->yaw_imu_angle = calc_rel_angle(gbal->yaw_imu_center, gbal->euler_angle.yaw);
		gbal->pitch_imu_angle = calc_rel_angle(gbal->pitch_imu_center, gbal->euler_angle.pitch);

		gbal->yaw_rel_angle = gbal->yaw_imu_angle;
		gbal->pitch_rel_angle = gbal->pitch_imu_angle;
	} else if (gbal->gimbal_motor_mode == ENCODE_MODE) {
		gimbal_update_ecd_angles(gbal, g_motors);

		gbal->yaw_rel_angle = gbal->yaw_ecd_angle;
		gbal->pitch_rel_angle = gbal->pitch_ecd_angle;
	} else {
		// TODO: Implement error handling / undefined motor mode state.
	}
}


void gimbal_update_targets(Gimbal_t *gbal, int16_t *g_channels) {
	if (gbal->gimbal_mode == IDLE_MODE) {
		gbal->yaw_target_angle = 0;
		gbal->pitch_target_angle = 0;
	} else if (gbal->gimbal_mode == AUTO_AIM_MODE) {
		float deltas[2];
		BaseType_t new_pack_response = get_message(AUTO_AIM, deltas, 0);
		if (new_pack_response == pdTRUE) {
			gbal->yaw_target_angle = gbal->yaw_rel_angle + deltas[0];
			gbal->pitch_target_angle = gbal->pitch_rel_angle + deltas[1];
		}
	} else if (gbal->gimbal_mode == PATROL_MODE && gbal->gimbal_act_mode == INDPET_MODE) {
		gbal->yaw_target_angle = 0;
	} else {
		float deltas[2];
		gimbal_calc_channels_to_angles(g_channels, deltas);

		gbal->yaw_target_angle -= deltas[0];
		if (gbal->yaw_target_angle > PI) gbal->yaw_target_angle -= 2.0f * PI;
		if (gbal->yaw_target_angle < -PI) gbal->yaw_target_angle += 2.0f * PI;

		gbal->pitch_target_angle -= deltas[1];
		if (gbal->pitch_target_angle < -20.0f * DEGREE2RAD) gbal->pitch_target_angle = -20.0f * DEGREE2RAD;
		if (gbal->pitch_target_angle > 12.0f * DEGREE2RAD) gbal->pitch_target_angle = 12.0f * DEGREE2RAD;
	}

	gimbal_set_limited_angle(gbal, gbal->yaw_target_angle, gbal->pitch_target_angle);
}


//float gimbal_calc_dual_pid_out(PID2_t *f_pid, PID2_t *s_pid, float f_cur_val, s_cur_val) {
//	return pid2_dual_loop_control(f_pid,
//			s_pid,
//			0,
//			f_cur_val,
//			s_cur_val,
//			GIMBAL_TASK_EXEC_TIME * 0.001,
//			GIMBAL_TASK_EXEC_TIME * 0.001);
//}


void gimbal_send_motor_volts(Motor_t *g_motors) {
	MotorSetMessage_t motor_set_message;
	motor_set_message.motor_can_volts[GIMBAL_YAW_CAN_ID] = g_motors[GIMBAL_YAW_MOTOR_INDEX].tx_data;
	motor_set_message.motor_can_volts[GIMBAL_PITCH_CAN_ID] = g_motors[GIMBAL_PITCH_MOTOR_INDEX].tx_data;
	motor_set_message.data_enable = (1 << GIMBAL_YAW_CAN_ID) | (1 << GIMBAL_PITCH_CAN_ID);
	pub_message(MOTOR_SET, &motor_set_message);
}

/******************************** For Comms Above **************************************/
/******************************** For Auto Aiming Below ********************************/
void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, UC_Auto_Aim_Pack_t *pack) {
//	float cur_yaw_target = 0.0;
//	float cur_pitch_target = 0.0;
//	float delta_yaw= 0.0;
//	float delta_pitch = 0.0;
//
//	if(gbal->gimbal_mode == PC_MODE){
//		/* filter applied here, TODO may add kalman filter here, depends on data input */
//		float filtered_delta_yaw = ewma_filter(&gbal->ewma_f_aim_yaw, pack->delta_yaw);
////		pack->yaw_data = sliding_window_mean_filter(&gbal->swm_f_aim_yaw, pack->yaw_data);
//		delta_yaw = in_out_map(filtered_delta_yaw, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI
//
//		float filtered_delta_pitch = ewma_filter(&gbal->ewma_f_aim_pitch, pack->delta_pitch);
////		pack->pitch_data = sliding_window_mean_filter(&gbal->swm_f_aim_pitch, pack->yaw_data);
//		delta_pitch = in_out_map(filtered_delta_pitch, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI
//	}
//	/* get the latest angle position of pitch and yaw motor */
//	gimbal_get_ecd_fb_data(&gimbal);
//	/* NOTE: Even if the target was beyond pi, the motor still tracked the same dir bc of spd loop and phase delay,
//	 * and right about next time, the feedback of motor would be changed from pi to -pi(or inverse), which will
//	 * also update the target into right scale of angle */
//	gimbal_update_rel_turns(gbal, GIMBAL_JUMP_THRESHOLD);
//	if(gbal->gimbal_motor_mode == GYRO_MODE){
////		 cur_yaw_target = gbal->yaw_cur_abs_angle - delta_yaw;
//		cur_yaw_target = gbal->final_abs_yaw - delta_yaw;
//		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
//		// cur_pitch_target = gbal->final_abs_pitch + delta_pitch * PITCH_GEAR_RATIO;
//	}
//	else{
////		cur_yaw_target = gbal->yaw_cur_rel_angle - delta_yaw;
//		cur_yaw_target = gbal->yaw_total_rel_angle - delta_yaw;
//		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
//	}
//	/* avoid small noise to spin the yaw */
//	if(fabs(delta_yaw)>= 1.0f*DEGREE2RAD)
//		gbal->yaw_tar_angle = cur_yaw_target;
//	if(fabs(delta_pitch)>= 1.0f*DEGREE2RAD)
//		gbal->pitch_tar_angle = cur_pitch_target;
//	/* independent mode don't allow set yaw angle */
////	if(gbal->gimbal_act_mode == INDPET_MODE)
////		gbal->yaw_tar_angle = 0;
}
/******************************** For Auto Aiming Above ********************************/

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void gimbal_cmd_exec(Gimbal_t *gbal, Motor_t *g_motors) {
	float yaw_diff = calc_rel_angle(gbal->yaw_target_angle, gbal->yaw_rel_angle);
	float pitch_diff = calc_rel_angle(gbal->pitch_target_angle, gbal->pitch_rel_angle);

	g_motors[GIMBAL_YAW_MOTOR_INDEX].tx_data = pid2_dual_loop_control(&(gbal->yaw_f_pid),
			&(gbal->yaw_s_pid),
			0,
			yaw_diff,
			g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_rpm,
			GIMBAL_TASK_EXEC_TIME * 0.001,
			GIMBAL_TASK_EXEC_TIME * 0.001);

//	g_motors[GIMBAL_PITCH_MOTOR_INDEX].tx_data = pid2_single_loop_control(&(gbal->pitch_f_pid), 0, pitch_diff, GIMBAL_TASK_EXEC_TIME * 0.001);
	g_motors[GIMBAL_PITCH_MOTOR_INDEX].tx_data = pid2_dual_loop_control(&(gbal->pitch_f_pid),
			&(gbal->pitch_s_pid),
			0,
			pitch_diff,
			g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_rpm,
			GIMBAL_TASK_EXEC_TIME * 0.001,
			GIMBAL_TASK_EXEC_TIME * 0.001);
}

#endif /* __GIMBAL_APP_C__ */
