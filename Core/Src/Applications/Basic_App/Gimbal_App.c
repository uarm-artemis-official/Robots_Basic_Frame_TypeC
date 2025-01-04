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
static UC_Auto_Aim_Pack_t temp_pack;
static Motor_t gimbal_motors[2];
static int32_t motor_tx_buffer[8];

/* With encoder mode, task execution time (per loop): 1ms */
void Gimbal_Task_Function(void const * argument)
{
    /* USER CODE BEGIN Gimbal_Task_Function */
	float temp_idle_yaw = 0.0;
	float temp_idle_pitch = 0.0;
	uint8_t gimbal_idle_flag = 1;

	/* gimbal task LD indicator */
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);

	/* init gimbal task */
	gimbal_task_init(&gimbal, gimbal_motors);

	/* reset calibration using ramp function */
	gimbal_calibration_reset(&gimbal, gimbal_motors);

	/* define after-detection delay var here */
//	int16_t gimbal_control_counter=0;

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(GIMBAL_TASK_EXEC_TIME); // task exec period 1ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		gimbal_get_rc_info(&gimbal);
		gimbal_get_motor_feedback(&gimbal, gimbal_motors);

		/* make sure offset already be set to the gyro */
		float imu_readings[2];
		BaseType_t new_imu_message = peek_message(IMU_READINGS, imu_readings, 0);
		if (new_imu_message == pdTRUE) {
			gimbal_gyro_update_abs_angle(&gimbal, imu_readings[0], imu_readings[1]);
		} else if(gimbal.gimbal_motor_mode == GYRO_MODE) {
			/* imu not ready -> deactivate gyro mode*/
			gimbal.gimbal_act_mode = IDLE_MODE;
		}

		/************************************* MODE SELECTION START *************************************/
		if(gimbal.gimbal_mode == IDLE_MODE){
			/* use ramp function to approximate zeros */
			gimbal_idle_flag = 1;
//			gimbal_get_ecd_fb_data(&gimbal);
			//		  gimbal_update_rel_turns(&gimbal, GIMBAL_JUMP_THRESHOLD);
			temp_idle_yaw = gimbal.yaw_cur_rel_angle + (0 - gimbal.yaw_cur_rel_angle)*ramp_calculate(&(gimbal.yaw_ramp));
			temp_idle_pitch = gimbal.pitch_cur_rel_angle + (0 - gimbal.pitch_cur_rel_angle)*ramp_calculate(&(gimbal.pitch_ramp));
			/* reset everything */
			gimbal_set_limited_angle(&gimbal, temp_idle_yaw, temp_idle_pitch);
			gimbal.yaw_total_rel_angle = gimbal.yaw_cur_rel_angle;
			gimbal.yaw_total_turns = 0;
			gimbal.yaw_turns_count = 0;
		} else {
			/* reset ramp counter for next use */
			gimbal_idle_flag = 0;
			gimbal.pitch_ramp.count = 0;
			gimbal.yaw_ramp.count = 0;
			temp_idle_yaw = 0;
			temp_idle_pitch = 0;

			/* Safely reset count after some greater value */
			if(gimbal.yaw_turns_count >= 60000)
				gimbal.yaw_turns_count = 0;
			if(gimbal.yaw_total_turns >= 60000)
				gimbal.yaw_total_turns = 0;

		}//None IDLE MODE else
		/* Update current mode */
		gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;

		/* set motor voltage through cascade pid controller */
		gimbal_cmd_exec(&gimbal, gimbal_motors, DUAL_LOOP_PID_CONTROL, gimbal_idle_flag);
		gimbal_send_motor_volts(gimbal_motors);

		/* update rel angle and send to chassis */
		//	 gimbal_update_comm_info(&gimbal, &gimbal_angle_message.message);
		gimbal_send_rel_angles(&gimbal);

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}

	/* USER CODE END Gimbal_Task_Function */
}//task ends

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
	gbal->gimbal_motor_mode = mode;
}

/*
 * @brief     the initialization process of the gimbal task,
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_task_init(Gimbal_t *gbal, Motor_t *g_motors) {
	/* Waiting for imu to be set normal temp */
//	osDelay(GIMBAL_INIT_TIME_MS);

	/* init motor pid */
	// angular pid based on radian(-pi, pi), speed pid based on rpm(-15000, 15000)
	motor_init(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), max_out_angle_yaw,  max_I_out_angle_yaw, max_err_angle_yaw, kp_angle_yaw, ki_angle_yaw, kd_angle_yaw,
					   max_out_spd_yaw, max_I_out_spd_yaw, max_err_spd_yaw, kp_spd_yaw, ki_spd_yaw, kd_spd_yaw,
					   kf_spd_yaw);//spd ff gain

	motor_init(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), max_out_angle_pitch,  max_I_out_angle_pitch, max_err_angle_pitch, kp_angle_pitch, ki_angle_pitch, kd_angle_pitch,
					     max_out_spd_pitch, max_I_out_spd_pitch, max_err_spd_pitch, kp_spd_pitch, ki_spd_pitch, kd_spd_pitch,
					     kf_spd_pitch);//spd ff gain

	/* set init gimbal mode */
	gimbal_set_board_mode(gbal, PATROL_MODE);     // patrol mode
	gimbal_set_act_mode(gbal, INDPET_MODE); // indepedent mode
	gimbal_set_motor_mode(gbal, ENCODE_MODE);

	/* reset gimbal data */
	gimbal_reset_data(gbal, g_motors);

	memset(motor_tx_buffer, 999999, 32);
}

/*
 * @brief     the initialization process of the gimbal task,
 * 			  centering and ranging the motors using ecd
 * @param[in] gimbal: main gimbal handler
 * */
uint8_t gimbal_cali_done_flag = 0;
void gimbal_calibration_reset(Gimbal_t *gbal, Motor_t *g_motors) {
	 /* reset the calibration flag first*/
	 gimbal_cali_done_flag = 0;

	 /* init ramp functions*/
	 ramp_init(&(gbal->yaw_ramp), 1500);//1.5s init
	 ramp_init(&(gbal->pitch_ramp), 1500);

	 float temp_pitch_ramp_output = 0.0f;
	 float temp_yaw_ramp_output   = 0.0f;

	 if (gbal->gimbal_motor_mode == ENCODE_MODE) {
		 for(;;){
			//get feedback first
//			gimbal_get_ecd_fb_data(gbal);
			gimbal_get_motor_feedback(gbal, g_motors);
			gimbal_update_rel_turns(gbal, GIMBAL_JUMP_THRESHOLD);
			temp_yaw_ramp_output = gbal->yaw_total_rel_angle + (0 - gbal->yaw_total_rel_angle)*ramp_calculate(&(gbal->yaw_ramp));
			temp_pitch_ramp_output   = gbal->pitch_cur_rel_angle + (0 - gbal->pitch_cur_rel_angle)*ramp_calculate(&(gbal->pitch_ramp));

			gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), temp_yaw_ramp_output, gbal->yaw_total_rel_angle);
			gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), temp_pitch_ramp_output, gbal->pitch_cur_rel_angle);
			gimbal_send_motor_volts(g_motors);

			 /* when the err of cali angle smaller */
			 if(fabs(gbal->yaw_total_rel_angle)< (2.0f * DEGREE2RAD)){ //|| counter >= 50000 /*timeout*/ //&& fabs(cur_pitch_radian)< (2.0f * DEGREE2RAD)
				 /* cali done */
	//			 motor_data[pitch_id].tx_data = 0;
	//			 motor_data[yaw_id].tx_data = 0;
				 gbal->pitch_ramp.count = 0;
				 gbal->yaw_ramp.count = 0;
				 gimbal_cali_done_flag = 1;
				 break;
			 }
			 osDelay(1);
		 }
	 } else {
		 Error_Handler();
	 }
}

/*
 * @brief     Reset all data internal gimbal struct
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_reset_data(Gimbal_t *gbal, Motor_t *g_motors) {
	motor_data_init(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]));
	motor_data_init(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]));

	memset(gbal, 0, sizeof(Gimbal_t));

	// Initialize non-zero Gimbal_t fields.
	gbal->yaw_ecd_center = YAW_ECD_CENTER; // center position of the yaw motor - encoder
	gbal->pitch_ecd_center = PITCH_ECD_CENTER;
	gbal->gyro_offset_slope = -1.84228e-10;
	gbal->euler_angle.timestamp = dwt_getCnt_us();

	init_folp_filter(&gbal->folp_f_yaw, 0.90f);
	init_folp_filter(&gbal->folp_f_pitch, 0.99f);

	init_ewma_filter(&gbal->ewma_f_x, 0.8f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_y, 0.8f);//0.6 for older client
	init_ewma_filter(&gbal->ewma_f_aim_yaw, 0.95f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_aim_pitch, 0.95f);//0.6 for older client

	init_swm_filter(&gbal->swm_f_x, 50);// window size 50
	init_swm_filter(&gbal->swm_f_y, 50);

	memset(&(gbal->ahrs_sensor), 0, sizeof(AhrsSensor_t));
	memset(&(gbal->euler_angle), 0, sizeof(Attitude_t));

	kalmanCreate(&(gbal->kalman_f), 0.001, 0.01);//0.0005 0.02
	gbal->prev_gimbal_motor_mode = ENCODE_MODE;
}
/******************  MODE SELECTION FUNCTIONS BELOW ********************/
void gimbal_get_raw_mpu_data(Gimbal_t *gbal, IMU_t *imu_hldr){
	memcpy(&(gbal->ahrs_sensor), &(imu_hldr->ahrs_sensor), sizeof(AhrsSensor_t));
}

/*
 * @brief     Copy the gyroscope data from imu and calculate quaternion
 * 			  and euler's angle through attitude-breakdown algorithms.
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_get_euler_angle(Gimbal_t *gbal){
	gimbal_get_raw_mpu_data(gbal, &imu); // copy data to avoid mem leaks
//	atti_math_calc(&gbal->ahrs_sensor, &gbal->euler_angle); //complementary filter parsed angle
//	mahony_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));	//mahony algo
	madgwick_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));  //madgwick algo
}

/*
 * @brief     Copy the gyroscope data from imu and calculate quaternion
 * 			  and euler's absolute angle through attitude-breakdown algorithms.
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_gyro_update_abs_angle(Gimbal_t *gbal, float yaw, float pitch) {
	 /* get timestamp */
	 uint32_t DWTcnt = dwt_getCnt_us();// systemclock_core 168MHz ->usec
//	 uint32_t delta_t = min(DWTcnt - gbal->euler_angle.timestamp, 3000);
	 /* filter the yaw angle data to handle shift */
	 gbal->euler_angle.yaw = first_order_low_pass_filter(&(gbal->folp_f_yaw), yaw);
	 gbal->euler_angle.pitch = first_order_low_pass_filter(&(gbal->folp_f_pitch), pitch);
//	 gbal->euler_angle.yaw = KalmanFilter(&(gbal->kalman_f), gbal->euler_angle.yaw);

	 /* apply an integral linear offset for yaw angle */
	 gbal->yaw_prev_angle = gbal->yaw_cur_abs_angle;
	 gbal->yaw_cur_abs_angle = gbal->euler_angle.yaw;
	 if(gbal->yaw_cur_abs_angle - gbal->yaw_prev_angle >= 5.5f) // 320 degrees
		gbal->yaw_total_turns++;
	 else if(gbal->yaw_cur_abs_angle - gbal->yaw_prev_angle <= -5.5f)
	 	gbal->yaw_total_turns--;
	 gbal->final_abs_yaw = gbal->yaw_cur_abs_angle - 2*PI*gbal->yaw_total_turns;

	 gbal->euler_angle.timestamp = DWTcnt;

	 /* apply first order filter to pitch angle */
	 gbal->pitch_cur_abs_angle = gbal->euler_angle.pitch;
	 if(gbal->pitch_cur_abs_angle - gbal->pitch_prev_angle >= 5.5f) // 300 degrees
	 	gbal->pitch_total_turns++;
	 else if(gbal->pitch_cur_abs_angle - gbal->pitch_prev_angle <= -5.5f)
		gbal->pitch_total_turns--;
	 gbal->final_abs_pitch = gbal->pitch_cur_abs_angle - 2*PI*gbal->pitch_total_turns;

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

/*
 * @brief     Get relative angle of gimbal motors.
 * @param[in] raw_ecd: abs yaw ecd angle from feedback
 * @param[in] center_offset: the center offset of ecd mode
 * */
int16_t gimbal_get_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset)
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
void gimbal_update_ecd_rel_angle(Gimbal_t *gbal, Motor_t *g_motors) {
	int16_t yaw_ecd_rel_angle = gimbal_get_ecd_rel_angle(g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_angle, gbal->yaw_ecd_center);
	int16_t pitch_ecd_rel_angle = gimbal_get_ecd_rel_angle(g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_angle, gbal->pitch_ecd_center);

	gbal->yaw_prev_rel_angle = gbal->yaw_cur_rel_angle;
	gbal->yaw_cur_rel_angle = in_out_map(yaw_ecd_rel_angle,-4095,4096,-PI,PI);
	gbal->pitch_cur_rel_angle = in_out_map(pitch_ecd_rel_angle,-4095,4096,-PI,PI);
}

/*
 * @brief     Ensure the mode switch safely
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_safe_mode_switch(Gimbal_t *gbal){
 if(gbal->prev_gimbal_motor_mode != gbal->gimbal_motor_mode) {
	 if(gbal->gimbal_motor_mode == GYRO_MODE){
		 gbal->yaw_tar_angle = gbal->yaw_cur_abs_angle; // Set the target as current angle
		 gbal->pitch_tar_angle = gbal->pitch_cur_rel_angle; //  to avoid spin
	 }
	 else if(gbal->gimbal_motor_mode == ENCODE_MODE){
		 gbal->yaw_tar_angle = gbal->yaw_cur_rel_angle;
		 gbal->pitch_tar_angle = gbal->pitch_cur_rel_angle;
	 }
   }
}

/******************  MODE SELECTION FUNCTIONS ABOVE ********************/

/*
 * @brief     Update the total turns using critical value determination,
 * 			  would be very helpful for controlling the gimbal when spinning.
 * @copyright This algorithm is open-sourced by SZU, China
 * @param[in] gbal: main gimbal handler
 * @param[in] halfc: The value of half cycle,
 * 						if gyro mode then PI
 * 						if encoder mode then 4096(after mapping is PI)
 * */
void gimbal_update_truns(Gimbal_t *gimbal, float halfc){
  // If the absolute difference in angle between the current and previous reading
  // is greater than or equal to half a circle (either PI or 4096, depending on the mode)
	if(fabs(gimbal->yaw_cur_rel_angle - gimbal->yaw_prev_angle) >= halfc){
	  // If the current relolute angle is less than the previous one
	  // the gimbal has rotated over the boundary in a counterclockwise direction.
		if(gimbal->yaw_cur_rel_angle < gimbal->yaw_prev_angle){
		  // Increase the turn count by the difference in angle across the boundary
			gimbal->yaw_turns_count += 2*halfc -  gimbal->yaw_prev_angle + gimbal->yaw_cur_rel_angle;
		}
		else{
		  // Otherwise, the gimbal has rotated over the boundary in a clockwise direction.
		  // Decrease the turn count by the difference in angle across the boundary
			gimbal->yaw_turns_count -= 2*halfc +  gimbal->yaw_prev_angle - gimbal->yaw_cur_rel_angle;
		}
	}
	else{
	  // If the relolute difference in angle between the current and previous reading
	  // is less than half a circle, add the difference to the turn count normally.
		gimbal->yaw_turns_count += gimbal->yaw_cur_rel_angle - gimbal->yaw_prev_angle;
	}
	// Update the previous angle for the next comparison
	gimbal->yaw_prev_angle = gimbal->yaw_cur_rel_angle;
}

void gimbal_update_rel_turns(Gimbal_t* gbal, int jump_threshold){
	if(gbal->yaw_cur_rel_angle - gbal->yaw_prev_rel_angle >= jump_threshold)
		gbal->yaw_turns_count++;
	 else if(gbal->yaw_cur_rel_angle - gbal->yaw_prev_rel_angle  <= -jump_threshold)
		gbal->yaw_turns_count--;
	 gbal->yaw_total_rel_angle = gbal->yaw_cur_rel_angle - 2*PI*gbal->yaw_turns_count;
}

/*
 * @brief     set the target angle with limited range
 * @param[in] gbal: main gimbal handler
 * @param[in] target yaw and pitch relative angle(-pi, pi)
 */
void gimbal_set_limited_angle(Gimbal_t *gbal, float yaw_target_angle, float pitch_target_angle){
	gbal->yaw_tar_angle = yaw_target_angle;
	gbal->pitch_tar_angle = pitch_target_angle;
	/* only set limit for yaw where is no slipring */
//	VAL_LIMIT(gbal->yaw_tar_angle,
//				   -PI,
//				    PI);
	/* set the limit for pitch */
	VAL_LIMIT(gbal->pitch_tar_angle,
				   -PITCH_GYRO_DELTA,
					PITCH_GYRO_DELTA);
}

/****************** rc data related below ********************/
/*
 * @brief     set the target speed
 * @param[in] gbal: main gimbal handler
 * @param[in] target yaw spd (-)
 */
void gimbal_set_spd(Gimbal_t *gbal, int16_t yaw_target_spd){
	gbal->yaw_tar_spd = yaw_target_spd;
    VAL_LIMIT(gbal->yaw_tar_spd, -5000, 5000);
}

#ifndef GIMBAL_MOTOR_DEBUG
/******************************** For Comms Below ********************************/
//static void gimbal_update_comm_info(Gimbal_t *gbal, CommMessageUnion_t *cmu){
//	float temp_angle = YAW_POSITIVE_DIR * gbal->yaw_cur_rel_angle * YAW_GEAR_RATIO;
//	cmu->comm_ga.angle_data[0] = temp_angle;
//	cmu->comm_ga.angle_data[1] = gbal->yaw_cur_abs_angle;
//	cmu->comm_ga.angle_data[2] = 0;
//	cmu->comm_ga.angle_data[3] = 0;
//	cmu->comm_ga.send_flag = 1;
//}

void gimbal_update_rc_rel_angle(Gimbal_t *gbal, float delta_yaw, float delta_pitch) {
	float cur_yaw_target = 0.0;
	float cur_pitch_target = 0.0;

	/* get the latest angle position of pitch and yaw motor */
//	gimbal_get_ecd_fb_data(&gimbal);
	/* Update rel turns and total angle */
	gimbal_update_rel_turns(gbal, GIMBAL_JUMP_THRESHOLD);
	/* NOTE: Even if the target was beyond pi, the motor still tracked the same dir bc of spd loop and phase delay,
	 * and right about next time, the feedback of motor would be changed from pi to -pi(or inverse), which will
	 * also update the target into right scale of angle */
	if(gbal->gimbal_motor_mode == GYRO_MODE){
//		 cur_yaw_target = gbal->yaw_cur_abs_angle - delta_yaw;
		cur_yaw_target = gbal->final_abs_yaw - delta_yaw;
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
		// cur_pitch_target = gbal->final_abs_pitch + delta_pitch * PITCH_GEAR_RATIO;
	}
	else{
//		cur_yaw_target = gbal->yaw_cur_rel_angle - delta_yaw;
		cur_yaw_target = gbal->yaw_total_rel_angle - delta_yaw;
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
	}
	/* avoid small noise to spin the yaw */
	if(fabs(delta_yaw)> 1.0f*DEGREE2RAD){
		gbal->yaw_tar_angle = cur_yaw_target;
	}
	if(fabs(delta_pitch)> 1.0f*DEGREE2RAD)
		gbal->pitch_tar_angle = cur_pitch_target;

//	if(gbal->gimbal_mode == PC_MODE)
//		gimbal_safe_mode_switch(gbal);// Safely switch mode

	/* independent mode don't allow set yaw angle */
	if(gbal->gimbal_act_mode == INDPET_MODE)
		gbal->yaw_tar_angle = 0;
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
	RCInfoMessage_t rc_info;
	BaseType_t new_rc_info_message = peek_message(RC_INFO, &rc_info, 0);
	if (new_rc_info_message == pdTRUE) {
		gimbal_set_modes(gbal, rc_info.modes);

		float delta_yaw = in_out_map((float) rc_info.channels[0],
				-CHANNEL_OFFSET_MAX_ABS_VAL,
				CHANNEL_OFFSET_MAX_ABS_VAL,
				-0.5*0.16667*PI, 0.5*0.16667*PI); //(-15d, 15d)
		float delta_pitch = in_out_map((float) rc_info.channels[1],
				-CHANNEL_OFFSET_MAX_ABS_VAL,
				CHANNEL_OFFSET_MAX_ABS_VAL,
				-0.39*0.16667*PI, 0.391*0.16667*PI); //(-12d, 12d)

		if (gbal->gimbal_mode != IDLE_MODE && (gbal->gimbal_mode != AUTO_AIM_MODE || temp_pack.target_num <= -1)) {
			gimbal_update_rc_rel_angle(gbal, delta_yaw, delta_pitch);
			gimbal_set_limited_angle(gbal, gbal->yaw_tar_angle, gbal->pitch_tar_angle);
		}
	}
}


void gimbal_get_motor_feedback(Gimbal_t *gbal, Motor_t *g_motors) {
	Motor_Feedback_t motor_feedback_buffer[MOTOR_COUNT];
	BaseType_t new_feedback_message = peek_message(MOTOR_READ, motor_feedback_buffer, 0);
	if (new_feedback_message == pdTRUE) {
		memcpy(&(g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback), &(motor_feedback_buffer[GIMBAL_YAW_CAN_ID]), sizeof(Motor_Feedback_t));
		memcpy(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback), &(motor_feedback_buffer[GIMBAL_PITCH_CAN_ID]), sizeof(Motor_Feedback_t));
	}
}


void gimbal_send_rel_angles(Gimbal_t *gbal) {
	CANCommMessage_t rel_angle_message;
	rel_angle_message.topic_name = GIMBAL_REL_ANGLES;
	memcpy(rel_angle_message.data, &(gbal->yaw_cur_rel_angle), sizeof(float));
	memcpy(&(rel_angle_message.data[4]), &(gbal->pitch_cur_rel_angle), sizeof(float));
	pub_message(COMM_OUT, &rel_angle_message);
}


void gimbal_calc_dual_pid_out(Motor_t *motor, float target, float f_cur_val) {
	motor->tx_data = pid_dual_loop_control(feedforward(&motor->motor_info.ff, target),//pid+ff
									  &(motor->motor_info.f_pid),
									  &(motor->motor_info.s_pid),
									  f_cur_val,
									  motor->motor_feedback.rx_rpm,
									  GIMBAL_TASK_EXEC_TIME * 0.001);
}


void gimbal_calc_single_pid_out(Motor_t *motor, float target) {
	motor->tx_data = pid_single_loop_control(target,
											&(motor->motor_info.s_pid),
											motor->motor_feedback.rx_rpm,
											GIMBAL_TASK_EXEC_TIME * 0.001);
}


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
#endif

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void gimbal_cmd_exec(Gimbal_t *gbal, Motor_t *g_motors, uint8_t mode, uint8_t idle_flag) {
	if(mode == DUAL_LOOP_PID_CONTROL) {
		/* set motor voltage through cascade pid controller */
		if (gbal->gimbal_motor_mode == ENCODE_MODE) {
			if (idle_flag == 1) {
				gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), gbal->yaw_tar_angle, gbal->yaw_cur_rel_angle);
				gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), gbal->pitch_tar_angle, gbal->pitch_cur_rel_angle);
			} else {
				gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), gbal->yaw_tar_angle, gbal->yaw_total_rel_angle);
				gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), gbal->pitch_tar_angle, gbal->pitch_cur_rel_angle);
			}
		} else if (gbal->gimbal_motor_mode == GYRO_MODE) {
			gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), gbal->yaw_tar_angle, gbal->final_abs_yaw);
			gimbal_calc_dual_pid_out(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), gbal->pitch_tar_angle, gbal->pitch_cur_rel_angle);
		} else {
			Error_Handler();
		}
	} else if(mode == SINGLE_LOOP_PID_CONTROL) { // only spd control
		gimbal_calc_single_pid_out(&(g_motors[GIMBAL_YAW_MOTOR_INDEX]), gbal->yaw_tar_spd);
		gimbal_calc_single_pid_out(&(g_motors[GIMBAL_PITCH_MOTOR_INDEX]), gbal->pitch_tar_spd);
	} else {
		g_motors[GIMBAL_YAW_MOTOR_INDEX].tx_data = 0;
		g_motors[GIMBAL_PITCH_MOTOR_INDEX].tx_data = 0;
	}
}

#endif /* __GIMBAL_APP_C__ */
