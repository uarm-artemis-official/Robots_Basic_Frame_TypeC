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

#include <Control_App.h>
#include <Gimbal_App.h>
#include <string.h>
#include "stdio.h"
#include "Comm_App.h"
#include "debugger.h"
#include "kalman_filters.h"
#include "maths.h"
#include "public_defines.h"
#include "angle_process.h"
#include "auto_aim.h"
#include "dwt.h"
#include "task.h"
#include <PC_UART_App.h>

extern uint8_t imu_init_flag;
extern float can_tx_scale_buffer[TOTAL_COMM_ID][4];
extern RemoteControl_t rc;
extern UC_Auto_Aim_Pack_t uc_auto_aim_pack;
extern uint8_t aa_pack_recv_flag;
//extern CommVision_t vision_pack;

/* define temp vision pack */
//CommVision_t temp_pack;
UC_Auto_Aim_Pack_t temp_pack;

/* declare internal function */
static void gimbal_update_rc_rel_angle(Gimbal_t *gbal, RemoteControl_t *rc_hdlr);
static void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, RemoteControl_t *rc_hdlr, UC_Auto_Aim_Pack_t *pack);

#ifdef GIMBAL_MOTOR_DEBUG
	/* select main tune mode */
//	#define AUTO_TUNE 1
	#define FINE_TUNE 2
	/* define single/dual loop tune */
//	#define SINGLE_LOOP_TUNE 3
	#define DUAL_LOOP_TUNE   4
	/* define tune pid structure */
	PID_t tune_pid_f;
	PID_t tune_pid_s;
	/* set a objective angle/spd */
	float ecd_target_angle;//-pi, pi
	int16_t ecd_target_spd;
	uint8_t target_motor_id;
	/* extern debugger counter to access timer */
	extern uint8_t debugger_signal_flag;
	extern uint16_t debugger_signal_counter;
#else
	static void gimbal_update_comm_info(Gimbal_t *gbal, CommMessageUnion_t *cmu);
	static void gimbal_rc_mode_selection(Gimbal_t* gbal, RemoteControl_t *rc_hdlr);
	static uint8_t dynamic_offset_center_flag = 0;
#endif

/* With encoder mode, task execution time (per loop): 1ms */
void Gimbal_Task_Function(void const * argument)
{

    /* USER CODE BEGIN Gimbal_Task_Function */
	/* gimbal task LD indicator */
	HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_SET);

	/* init gimbal task */
	gimbal_task_init(&gimbal);

	/* reset calibration using ramp function */
	gimbal_calibration_reset(&gimbal);

	/* define after-detection delay var here */
//	int16_t gimbal_control_counter=0;

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // task exec period 1ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

#ifndef GIMBAL_MOTOR_DEBUG //for only test temp closed loop control
	for(;;){

	  /* mode selection */
	  gimbal_rc_mode_selection(&gimbal, &rc);

	  /* make sure offset already be set to the gyro */
	  if(imu_init_flag == 1)
		 /* update gyroscope angle */
		 gimbal_gyro_update_abs_angle(&gimbal);
	  else if(gimbal.gimbal_motor_mode == GYRO_MODE)
		  /* imu not ready -> deactivate gyro mode*/
		  gimbal.gimbal_act_mode = IDLE_MODE;

	  /************************************* MODE SELECTION START *************************************/
	  if(gimbal.gimbal_mode == IDLE_MODE){
		  /* use ramp function to approximate zeros */
		  gimbal_get_ecd_fb_data(&gimbal,
		  						   &(motor_data[yaw_id].motor_feedback),
		  						   &(motor_data[pitch_id].motor_feedback));
		  float temp_idle_yaw = gimbal.yaw_cur_rel_angle + (0-gimbal.yaw_cur_rel_angle)*ramp_calculate(&gimbal.yaw_ramp);
		  float temp_idle_pitch = gimbal.pitch_cur_rel_angle + (0-gimbal.pitch_cur_rel_angle)*ramp_calculate(&gimbal.pitch_ramp);
		  /* reset everything */
		  gimbal_set_limited_angle(&gimbal, temp_idle_yaw, temp_idle_pitch);
		  /* turn off dynamic center offset */
		  dynamic_offset_center_flag = 0;
	  }
	  else{
		  /* reset ramp counter for next use */
		  gimbal.pitch_ramp.count = 0;
		  gimbal.yaw_ramp.count = 0;

		 if(dynamic_offset_center_flag == 0){
			for(int j=0;j<2;j++){
				/* < Dynamic Center Offset >
				* First loop: obtains the relative angle of the current coordinate
				* 			  system and resets the center offset, but introduces
				* 			  the original center error;
				* Second loop: continues the same operation, while the original
				* 			   center error introduced last time can be corrected.
				* */
				//FIXME: This can be removed by precisely adjusting PID parameters of motor
				gimbal_update_rc_rel_angle(&gimbal, &rc);//update current center value
				gimbal.yaw_ecd_center = gimbal.yaw_ecd_fb.rx_angle;//reset center value
				gimbal.yaw_cur_rel_angle = 0;//reset current rel angle to 0
				gimbal.pitch_ecd_center = gimbal.pitch_ecd_fb.rx_angle;//reset center value
				gimbal.pitch_cur_rel_angle = 0;//reset current rel angle to 0
			}
			dynamic_offset_center_flag = 1;
		 }

		 if(aa_pack_recv_flag == 1){
			 memcpy(&temp_pack, &uc_auto_aim_pack, sizeof(UC_Auto_Aim_Pack_t));
			 aa_pack_recv_flag = 0;
		 }
		 else{
//			 memcpy(&temp_pack, 0, sizeof(UC_Auto_Aim_Pack_t));
			 temp_pack.delta_pitch = 0;
			 temp_pack.delta_yaw = 0;
		 }
//		 memcpy(&temp_pack, &uc_auto_aim_pack, sizeof(UC_Auto_Aim_Pack_t));
		 /* if operator wants to activate auto-aim AND the camera has detected the object */
		 if(gimbal.gimbal_mode == AUTO_AIM_MODE && temp_pack.target_num > -1){
//			 if( gimbal.prev_gimbal_act_mode != gimbal.gimbal_act_mode){
//					 gimbal.yaw_tar_angle = gimbal.yaw_cur_abs_angle;
//					 gimbal.pitch_tar_angle = gimbal.pitch_cur_rel_angle;
//			 }
			 /* activate auto aiming */
			 gimbal_update_autoaim_rel_angle(&gimbal, &rc, &temp_pack);
			 /* set limited target angle */
			 gimbal_set_limited_angle(&gimbal, gimbal.yaw_tar_angle, gimbal.pitch_tar_angle);

		 }

		 /* artificial targeting */
		 else if(gimbal.gimbal_act_mode == GIMBAL_FOLLOW || gimbal.gimbal_act_mode == INDPET_MODE){
//			 if( gimbal.prev_gimbal_act_mode != gimbal.gimbal_act_mode){
//					 gimbal.yaw_tar_angle = gimbal.yaw_cur_rel_angle;
//					 gimbal.pitch_tar_angle = gimbal.pitch_cur_rel_angle;
//			 }
			 /* update gimbal rel angle */
			gimbal_update_rc_rel_angle(&gimbal, &rc);
			/* set limited target angle */
			gimbal_set_limited_angle(&gimbal, gimbal.yaw_tar_angle, gimbal.pitch_tar_angle);
//			printf("%f,%f\r\n", gimbal.pitch_tar_angle, gimbal.pitch_cur_rel_angle);
//			gimbal.prev_gimbal_act_mode = gimbal.gimbal_act_mode;

		 }

		 else if(gimbal.gimbal_act_mode == GIMBAL_CENTER){
//			 if( gimbal.prev_gimbal_act_mode != SELF_GYRO || gimbal.prev_gimbal_act_mode != GIMBAL_CENTER){
//				 gimbal.yaw_tar_angle = gimbal.yaw_cur_abs_angle;
//				 gimbal.pitch_tar_angle = gimbal.pitch_cur_rel_angle;
//			 }
			/* update gimbal rel angle */
			gimbal_update_rc_rel_angle(&gimbal, &rc);
			/* set limited target angle */
			gimbal_set_limited_angle(&gimbal, gimbal.yaw_tar_angle, gimbal.pitch_tar_angle);

//			gimbal.prev_gimbal_act_mode = GIMBAL_CENTER;
		 }

		 else if(gimbal.gimbal_act_mode == SELF_GYRO){
//			 if( gimbal.prev_gimbal_act_mode != SELF_GYRO || gimbal.prev_gimbal_act_mode != GIMBAL_CENTER){
//				 gimbal.yaw_tar_angle = gimbal.yaw_cur_abs_angle;
//				 gimbal.pitch_tar_angle = gimbal.pitch_cur_rel_angle;
//			 }
			 if(gimbal.gimbal_motor_mode == GYRO_MODE){
				/* update gimbal rel ecd angle for pitch */
				gimbal_update_rc_rel_angle(&gimbal, &rc);
				/* set limited target angle */
				gimbal_set_limited_angle(&gimbal, gimbal.yaw_tar_angle, gimbal.pitch_tar_angle);


			}
//			 gimbal.prev_gimbal_act_mode = SELF_GYRO;
		}
	}//None IDLE MODE else

	 /* set motor voltage through cascade pid controller */
	 gimbal_cmd_exec(&gimbal, DUAL_LOOP_PID_CONTROL);

	 /* update rel angle and send to chassis */
	 gimbal_update_comm_info(&gimbal, &gimbal_angle_message.message);

	 /* delay until wake time */
	 vTaskDelayUntil(&xLastWakeTime, xFrequency);
   }

	/* USER CODE END Gimbal_Task_Function */
}//task ends
#else
	/* *****************************************************************************************************
	 * init tune pid structure as needed
	 *
	 * Generally, for auto tune, we need to set the outer loop as 0(no outer loop);
	 * 	          for fine tune, just copy the auto tuned parameters
	 *
	 * @Inner loop: loop closer to the actual motor control, use the output of the outer loop as the input.
	 * 			   In this case, spd loop (require quick response), usually PD controller
	 * @Outer loop: loop to guide the system, use the system feedback as the input.
	 * 			   In this case, angular/position loop (require precision), usually PI + Deadzone controller
	 *
	 * ******************************************************************************************************/
	motor_debug_init(yaw_id, max_out_angle_yaw,  max_I_out_angle_yaw, max_err_angle_yaw, kp_angle_yaw, ki_angle_yaw, kd_angle_yaw,
					   max_out_spd_yaw, max_I_out_spd_yaw, max_err_spd_yaw, kp_spd_yaw, ki_spd_yaw, kd_spd_yaw,
					   0);// no need spd ff gain
//	motor_debug_init(pitch_id, max_out_angle_pitch,  max_I_out_angle_pitch, max_err_angle_pitch, kp_angle_pitch, ki_angle_pitch, kd_angle_pitch,
//					     max_out_spd_pitch, max_I_out_spd_pitch, max_err_spd_pitch, kp_spd_pitch, ki_spd_pitch, kd_spd_pitch,
//					     0);//no need spd ff gain
	target_motor_id = yaw_id;
	ecd_target_angle = 0.5f*PI;

	/* debug loop begins */
	for(;;){
		/* use rc switch to engage a debug session */
		if(rc.ctrl.s1 == SW_DOWN){
#ifdef FINE_TUNE
			if(imu_init_flag == 1)
				 /* update gyroscope angle */
				 gimbal_gyro_update_abs_angle(&gimbal);

	#ifdef SINGLE_LOOP_TUNE
			 /* update the recc pid params */
			 motor_debug_init(target_motor_id, max_out_angle_yaw,  max_I_out_angle_yaw, max_err_angle_yaw, 0, 0, 0,
								   max_out_spd_yaw, max_I_out_spd_yaw, max_err_spd_yaw, tune_pid_s.kp, tune_pid_s.ki, tune_pid_s.kd,
								   0);//set spd ff gain to 0, fine tune ff gain later

			  /* fine tune the pid parameters */
			  gimbal_set_limited_angle(&gimbal, ecd_target_angle, 0);

			  /* update rel angle and send to chassis */
			  gimbal_get_ecd_fb_data(&gimbal,
									 &(motor_data[yaw_id].motor_feedback),
									 &(motor_data[pitch_id].motor_feedback));

			  /* set motor voltage through cascade pid controller */
			  motor_spid_step_signal_generator(0, gimbal.yaw_tar_angle, gimbal.yaw_cur_rel_angle, target_motor_id, &tune_pid_s);
//			  gimbal_cmd_exec(&gimbal, DUAL_LOOP_PID_CONTROL);

	#elif defined(DUAL_LOOP_TUNE)

			  /* update the recc pid params */

			   /* fine tune the pid parameters */
			   gimbal_set_limited_angle(&gimbal, ecd_target_angle, ecd_target_angle);

			   /* update rel angle and send to chassis */
			   gimbal_get_ecd_fb_data(&gimbal,
									 &(motor_data[yaw_id].motor_feedback),
									 &(motor_data[pitch_id].motor_feedback));

			   /* set motor voltage through cascade pid controller */
			   motor_fpid_step_signal_generator(0, gimbal.pitch_tar_angle, target_motor_id, ENCODE_MODE);

			   printf("%ld,%d\r\n", motor_data[target_motor_id].tx_data, motor_data[target_motor_id].motor_feedback.rx_rpm);
	#endif
#elif defined(AUTO_TUNE)// no outer angle loop
		 /* auto tune raw pid from matlab system identification */
			  gimbal_get_ecd_fb_data(&gimbal,
									 &(motor_data[yaw_id].motor_feedback),
									 &(motor_data[pitch_id].motor_feedback));
			  /*generate step signal, shaking head */
			  //FIXME: Currently we generate step signal which is not precise enough. We need to use sine signnal to fit data
			  if(debugger_signal_counter < 5){//500 ms period
				  /* generate periodical tx data */
				  motor_step_signal_generator(0, (int16_t)PI*1000, target_motor_id);
				  /* update rel angle and send to chassis */
				  gimbal_get_ecd_fb_data(&gimbal,
										 &(motor_data[yaw_id].motor_feedback),
										 &(motor_data[pitch_id].motor_feedback));
				  /* use uart to transmit data to upper computer */
				  printf("%ld %d\r\n", motor_data[target_motor_id].tx_data, motor_data[target_motor_id].motor_feedback.rx_rpm);
			  }
			  else if(debugger_signal_counter < 10){
				  motor_step_signal_generator(0, (int16_t)-PI*1000, target_motor_id);
				  /* update rel angle and send to chassis */
				  gimbal_get_ecd_fb_data(&gimbal,
										 &(motor_data[yaw_id].motor_feedback),
										 &(motor_data[pitch_id].motor_feedback));
				  printf("%ld %d\r\n", motor_data[target_motor_id].tx_data, motor_data[target_motor_id].motor_feedback.rx_rpm);
			  }
			  else
				  debugger_signal_counter = 0;// reset debugger counter so that the head shaking again

#endif

		}
		else{
			  debugger_signal_flag = 0;
			  debugger_signal_counter = 0;
			   /* update rel angle and send to chassis */
			   gimbal_get_ecd_fb_data(&gimbal,
									 &(motor_data[yaw_id].motor_feedback),
									 &(motor_data[pitch_id].motor_feedback));
#ifdef FINE_TUNE
			  gimbal_set_limited_angle(&gimbal, 0, 0);
			  gimbal_cmd_exec(&gimbal, DUAL_LOOP_PID_CONTROL);
#elif defined(AUTO_TUNE)
			  motor_data[yaw_id].tx_data = 0;
#endif
		}
		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
#endif

/*
 * @brief     set the gimbal board work mode:
 * 				patrol | detected armor | Auto_Poilt | IDLE(no action) | Debug(remote control)
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: Board work mode
 * */
void gimbal_set_mode(Gimbal_t *gbal, BoardMode_t mode){
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
void gimbal_task_init(Gimbal_t *gbal){
	/* reset rc data */
	rc.ctrl.ch0 = 0;
	rc.ctrl.ch1 = 0;
	rc.ctrl.ch2 = 0;
	rc.ctrl.ch3 = 0;
	rc.ctrl.s1 = SW_MID;
	rc.ctrl.s2 = SW_MID;

	/* Waiting for imu to be set normal temp */
	osDelay(GIMBAL_INIT_TIME_MS);

	/* init motor pid */
	// angular pid based on radian(-pi, pi), speed pid based on rpm(-15000, 15000)
	motor_init(yaw_id, max_out_angle_yaw,  max_I_out_angle_yaw, max_err_angle_yaw, kp_angle_yaw, ki_angle_yaw, kd_angle_yaw,
					   max_out_spd_yaw, max_I_out_spd_yaw, max_err_spd_yaw, kp_spd_yaw, ki_spd_yaw, kd_spd_yaw,
					   kf_spd_yaw);//spd ff gain
	motor_init(pitch_id, max_out_angle_pitch,  max_I_out_angle_pitch, max_err_angle_pitch, kp_angle_pitch, ki_angle_pitch, kd_angle_pitch,
					     max_out_spd_pitch, max_I_out_spd_pitch, max_err_spd_pitch, kp_spd_pitch, ki_spd_pitch, kd_spd_pitch,
					     kf_spd_pitch);//spd ff gain

	/* set init gimbal mode */
	gimbal_set_mode(gbal, PATROL_MODE);     // patrol mode
	gimbal_set_act_mode(gbal, INDPET_MODE); // indepedent mode
	gimbal_set_motor_mode(gbal, ENCODE_MODE);

	/* reset gimbal data */
	gimbal_reset_data(gbal);

	/* set comm packs init target number */
	gimbal_angle_message.message.vision.target_num = 0;
}

/*
 * @brief     the initialization process of the gimbal task,
 * 			  centering and ranging the motors using ecd
 * @param[in] gimbal: main gimbal handler
 * */
uint8_t gimbal_cali_done_flag = 0;
void gimbal_calibration_reset(Gimbal_t *gbal){
	 /* reset the calibration flag first*/
	 gimbal_cali_done_flag = 0;
	 uint16_t counter = 0;

	 /* init ramp functions*/
	 ramp_init(&(gbal->yaw_ramp), 1500);//1.5s init
	 ramp_init(&(gbal->pitch_ramp), 1500);

	 /* created temp var to store the ramp calc value */
	 // use local variables so it won't affect the process angle
	 // so we can reset at any time
	 float temp_pitch_ramp_output = 0.0f;
	 float temp_yaw_ramp_output   = 0.0f;
	 float cur_pitch_radian       = 0.0f;
	 float cur_yaw_radian         = 0.0f;

	 for(;;){
		 //get feedback first
		 gimbal_get_ecd_fb_data(gbal,
								&(motor_data[yaw_id].motor_feedback),
								&(motor_data[pitch_id].motor_feedback));
		 cur_pitch_radian = in_out_map(gbal->pitch_ecd_fb.rx_angle, -4095, 4096, -PI, PI);
		 cur_yaw_radian   = in_out_map(gbal->yaw_ecd_fb.rx_angle, -4095, 4096, -PI, PI);
		 //apply ramp algo's form: ramp_out = cur + (tar - cur)*ramp_calc
		 //all the angles here are relative angle
		 temp_pitch_ramp_output = cur_pitch_radian + (0 - cur_pitch_radian)*ramp_calculate(&(gbal->pitch_ramp));
		 temp_yaw_ramp_output   = cur_yaw_radian + (0 - cur_yaw_radian)*ramp_calculate(&(gbal->yaw_ramp));
		 set_motor_can_volt( temp_yaw_ramp_output,
				 	 	 	 temp_pitch_ramp_output,
							 0, 0,
							 DUAL_LOOP_PID_CONTROL,
							 gimbal.gimbal_motor_mode);
		 counter++;
		 /* when the err of cali angle smaller */
		 if(fabs(cur_pitch_radian)< (2.0f * DEGREE2RAD)){ //|| counter >= 50000 /*timeout*/ //&& fabs(cur_pitch_radian)< (2.0f * DEGREE2RAD)
			 /* cali done */
			 motor_data[pitch_id].tx_data = 0;
			 motor_data[yaw_id].tx_data = 0;
			 gbal->pitch_ramp.count = 0;
			 gbal->yaw_ramp.count = 0;
			 gimbal_cali_done_flag = 1;
			 break;
		 }
		 osDelay(1);
	 }
//	 HAL_GPIO_WritePin(GPIOG, LD_C_Pin, RESET);
}

/*
 * @brief     Reset all data internal gimbal struct
 * @param[in] gimbal: main gimbal handler
 * */
void gimbal_reset_data(Gimbal_t *gbal){
	gbal->yaw_ang_rate = 0.0f;			//not used
	gbal->pitch_ang_rate = 0.0f;        //not used
	gbal->yaw_speed_rate = 0.0f;		//not used
	gbal->pitch_speed_rate = 0.0f;		//not used

	gbal->yaw_cur_abs_angle = 0.0f;
	gbal->yaw_prev_angle = 0.0f;
	gbal->pitch_cur_abs_angle = 0.0f;
	gbal->pitch_prev_angle = 0.0f;

	gbal->yaw_total_turns = 0;
	gbal->pitch_total_turns = 0;
	gbal->final_abs_yaw = 0;
	gbal->final_abs_pitch = 0;

	gbal->yaw_cur_rel_angle = 0.0f;
	gbal->pitch_cur_rel_angle = 0.0f;

	gbal->yaw_turns_count = 0;
	gbal->yaw_ecd_center = YAW_ECD_CENTER;					//center position of the yaw motor - encoder
	gbal->pitch_ecd_center = PITCH_ECD_CENTER;

	gbal->gyro_offset_slope = -1.84228e-10;
	gbal->gyro_offset_count = 0;
	gbal->euler_angle.timestamp = dwt_getCnt_us();

	gbal->yaw_tar_angle = 0.0f;
	gbal->pitch_tar_angle = 0.0f;
	gbal->yaw_tar_spd = 0.0f;
	gbal->pitch_tar_spd = 0.0f;

	gbal->axis.vx = 0;
	gbal->axis.vy = 0;
	gbal->axis.wz = 0;

	init_folp_filter(&gbal->folp_f_yaw, 0.90f);
	init_folp_filter(&gbal->folp_f_pitch, 0.99f);


	init_ewma_filter(&gbal->ewma_f_x, 0.50f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_y, 0.50f);//0.6 for older client
	init_ewma_filter(&gbal->ewma_f_aim_yaw, 0.95f);//0.65 for older client
	init_ewma_filter(&gbal->ewma_f_aim_pitch, 0.95f);//0.6 for older client

	init_swm_filter(&gbal->swm_f_x, 50);// window size 50
	init_swm_filter(&gbal->swm_f_y, 50);

	memset(&(gbal->ahrs_sensor), 0, sizeof(AhrsSensor_t));
	memset(&(gbal->euler_angle), 0, sizeof(Attitude_t));
	memset(&(gbal->yaw_ecd_fb), 0, sizeof(Motor_Feedback_Data_t));
	memset(&(gbal->pitch_ecd_fb), 0, sizeof(Motor_Feedback_Data_t));

	kalmanCreate(&(gbal->kalman_f), 0.001, 0.01);//0.0005 0.02
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
void gimbal_gyro_update_abs_angle(Gimbal_t *gbal){
	 /* get timestamp */
	 uint32_t DWTcnt = dwt_getCnt_us();// systemclock_core 168MHz ->usec
	 uint32_t delta_t = DWTcnt - gbal->euler_angle.timestamp;
	 if(delta_t < 3000){
         delta_t = 3000;//random setting, avoid overflow
     /* Cumulative number of compensation counts */
//     gbal->gyro_offset_count += 1;
	 }
	 gimbal_get_euler_angle(gbal);
	 /* filter the yaw angle data to handle shift */
	 gbal->euler_angle.yaw = first_order_low_pass_filter(&(gbal->folp_f_yaw), gbal->euler_angle.yaw);
	 gbal->euler_angle.pitch = first_order_low_pass_filter(&(gbal->folp_f_pitch), gbal->euler_angle.pitch);
//	 gbal->euler_angle.yaw = KalmanFilter(&(gbal->kalman_f), gbal->euler_angle.yaw);

	 /* apply an integral linear offset for yaw angle */
	 gbal->yaw_prev_angle = gbal->yaw_cur_abs_angle;
	 gbal->yaw_cur_abs_angle = gbal->euler_angle.yaw; //- delta_t *gbal->gyro_offset_slope*gbal->gyro_offset_count;
	 if(gbal->yaw_cur_abs_angle - gbal->yaw_prev_angle >= 5.0f) // 300 degrees
		 gbal->yaw_total_turns++;
	 else if(gbal->yaw_cur_abs_angle - gbal->yaw_prev_angle <= -5.0f)
	 	 gbal->yaw_total_turns--;
	 gbal->final_abs_yaw = gbal->yaw_cur_abs_angle - PI*gbal->yaw_total_turns;

	 gbal->euler_angle.timestamp = DWTcnt;
//	 printf("%d %f %d\r\n", (gbal->euler_angle.timestamp), gbal->euler_angle.yaw, delta_t); //export data to fit offset slope

	 /* update the turns */
//	 gimbal_update_turns(gbal, PI);
	 /* apply first order filter to pitch angle */
//	 gbal->euler_angle.pitch = first_order_low_pass_filter(&(gbal->folp_f), gbal->euler_angle.pitch);
	 gbal->pitch_cur_abs_angle = gbal->euler_angle.pitch;
	 if(gbal->pitch_cur_abs_angle - gbal->pitch_prev_angle >= 5.0f) // 300 degrees
	 		 gbal->pitch_total_turns++;
	 else if(gbal->pitch_cur_abs_angle - gbal->pitch_prev_angle <= -5.0f)
		 gbal->pitch_total_turns--;
	 gbal->final_abs_pitch = gbal->pitch_cur_abs_angle - PI*gbal->pitch_total_turns;
	 /* update angular velocity */
}
/*
 * @brief     update the relevant encoder angle
 * @param[in] gbal: main gimbal handler
 * */
void gimbal_get_ecd_fb_data(Gimbal_t *gbal, Motor_Feedback_Data_t *yaw_motor_fb, Motor_Feedback_Data_t *pitch_motor_fb){
	memcpy(&(gbal->yaw_ecd_fb), yaw_motor_fb, sizeof(Motor_Feedback_Data_t));
	gbal->yaw_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->yaw_ecd_fb.rx_angle, gbal->yaw_ecd_center);
	memcpy(&(gbal->pitch_ecd_fb), pitch_motor_fb, sizeof(Motor_Feedback_Data_t));
	gbal->pitch_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->pitch_ecd_fb.rx_angle, gbal->pitch_ecd_center);
	gimbal_update_ecd_rel_angle(gbal);
}
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
void gimbal_update_ecd_rel_angle(Gimbal_t *gbal){
	gbal->yaw_cur_rel_angle = in_out_map(gbal->yaw_ecd_fb.rx_angle,-4095,4096,-PI,PI);
	gbal->pitch_cur_rel_angle = in_out_map(gbal->pitch_ecd_fb.rx_angle,-4095,4096,-PI,PI);
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
/* rc data related below */
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
static void gimbal_update_comm_info(Gimbal_t *gbal, CommMessageUnion_t *cmu){
	float temp_angle = YAW_POSITIVE_DIR * gbal->yaw_cur_rel_angle * YAW_GEAR_RATIO;
	cmu->comm_ga.angle_data[0] = -temp_angle;//the direction of this are inverse.
	cmu->comm_ga.angle_data[1] = gbal->yaw_cur_abs_angle;
	cmu->comm_ga.angle_data[2] = 0;
	cmu->comm_ga.angle_data[3] = 0;
	cmu->comm_ga.send_flag = 1;
}

static void gimbal_update_rc_rel_angle(Gimbal_t *gbal, RemoteControl_t *rc_hdlr){
	float cur_yaw_target = 0.0;
	float cur_pitch_target = 0.0;
	float delta_yaw= 0.0;
	float delta_pitch = 0.0;
	//FIXME: not memcpy, may overwrite previous data, only test
	/* get the latest delta angle of pitch and yaw motor */
	if(rc_hdlr->control_mode == CTRLER_MODE){
		//TODO fine tune the precision of the controller
		delta_yaw = in_out_map(rc_hdlr->ctrl.ch0, -CHANNEL_OFFSET_MAX_ABS_VAL, CHANNEL_OFFSET_MAX_ABS_VAL,
										-0.5*0.16667*PI, 0.5*0.16667*PI);//(-15d, 15d)
		delta_pitch = in_out_map(rc_hdlr->ctrl.ch1, -CHANNEL_OFFSET_MAX_ABS_VAL, CHANNEL_OFFSET_MAX_ABS_VAL,
										-0.39*0.16667*PI, 0.391*0.16667*PI);//(-12d, 12d)
	}
	else if(rc_hdlr->control_mode == PC_MODE){
		//TODO fine tune the precision of the mouse
		/* expotional filter applied here */
		rc_hdlr->pc.mouse.x = ewma_filter(&gbal->ewma_f_x, rc_hdlr->pc.mouse.x);
//		rc_hdlr->pc.mouse.x = sliding_window_mean_filter(&gbal->swm_f_x, rc_hdlr->pc.mouse.x);
		delta_yaw = in_out_map(rc_hdlr->pc.mouse.x, -MOUSE_MAX_SPEED_VALUE, MOUSE_MAX_SPEED_VALUE,
												-30*PI, 30*PI);// 1000 -> 2*pi, old value +-30*PI
		rc_hdlr->pc.mouse.y = ewma_filter(&gbal->ewma_f_y, rc_hdlr->pc.mouse.y);
//		rc_hdlr->pc.mouse.y = sliding_window_mean_filter(&gbal->swm_f_y, rc_hdlr->pc.mouse.y);
		delta_pitch = in_out_map(rc_hdlr->pc.mouse.y, -MOUSE_MAX_SPEED_VALUE, MOUSE_MAX_SPEED_VALUE,
												-30*PI, 30*PI);// 1000 -> 2*pi, old value +-30*PI
	}
	/* get the latest angle position of pitch and yaw motor */
	gimbal_get_ecd_fb_data(&gimbal,
						   &(motor_data[yaw_id].motor_feedback),
						   &(motor_data[pitch_id].motor_feedback));
	/* NOTE: Even if the target was beyond pi, the motor still tracked the same dir bc of spd loop and phase delay,
	 * and right about next time, the feedback of motor would be changed from pi to -pi(or inverse), which will
	 * also update the target into right scale of angle */
	if(gbal->gimbal_motor_mode == GYRO_MODE){
		cur_yaw_target = gbal->yaw_cur_abs_angle - delta_yaw;
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
	}
	else{
		cur_yaw_target = gbal->yaw_cur_rel_angle - delta_yaw;
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
	}
	/* avoid small noise to spin the yaw */
	if(fabs(delta_yaw)> 1.0f*DEGREE2RAD){
		gbal->yaw_tar_angle = cur_yaw_target;
	}
	if(fabs(delta_pitch)> 1.0f*DEGREE2RAD)
		gbal->pitch_tar_angle = cur_pitch_target;
	/* independent mode don't allow set yaw angle */
	if(gbal->gimbal_act_mode == INDPET_MODE)
		gbal->yaw_tar_angle = 0;
}

/*
 * @brief     mode selection based on remote controller
 * @param[in] chassis: main chassis handler
 * @param[in] rc: main remote controller handler
 * */

static void gimbal_rc_mode_selection(Gimbal_t* gbal, RemoteControl_t *rc_hdlr){
#ifndef MANUAL_SET_GIMBAL_MODES
	BoardMode_t    board_mode = IDLE_MODE;
	BoardActMode_t act_mode   = INDPET_MODE;
	GimbalMotorMode_t motor_mode = ENCODE_MODE;
	if(rc_hdlr->control_mode == CTRLER_MODE){
		if(rc_hdlr->ctrl.s1 == SW_MID){
			/* if s1 down, then just shut down everything */
			board_mode = IDLE_MODE;
		}
		else{
			/* else just set up to patrol mode */
			board_mode = PATROL_MODE;
			if(rc_hdlr->ctrl.s1 == SW_UP){
				/* chassis follow gimbal center while follow yaw axis */
				act_mode = GIMBAL_CENTER;
				motor_mode = GYRO_MODE;
				if(rc_hdlr->ctrl.s1 == SW_UP && rc_hdlr->ctrl.s2 == SW_DOWN){
					/* spinning chassis while follow yaw axis */
					act_mode = SELF_GYRO;
					motor_mode = GYRO_MODE;//ENCODE_MODE
				}
			}
			else if(rc_hdlr->ctrl.s1 == SW_DOWN){
				/* chassis only follow yaw axis */
				act_mode = GIMBAL_FOLLOW;
				motor_mode = ENCODE_MODE;
				if(rc_hdlr->ctrl.s1 == SW_DOWN && rc_hdlr->ctrl.s2 == SW_DOWN){
					/* independent mode */
					act_mode = INDPET_MODE;
					motor_mode = ENCODE_MODE;
				}
			}
		}
	}

	/* pc mode selection */
	else if(rc_hdlr->control_mode == PC_MODE){
		/* from comm rc pack to obtain mode */
		board_mode = gbal->gimbal_mode;
		act_mode = gbal->gimbal_act_mode;

		/* update motor mode */
		if(rc_hdlr->pc.mouse.right_click.status == PRESSED)
			board_mode = AUTO_AIM_MODE;
		else{
			board_mode = gbal->gimbal_mode;
		}

		if(act_mode == GIMBAL_FOLLOW){
			motor_mode = ENCODE_MODE;
		}
		else if(act_mode == INDPET_MODE){
			motor_mode = ENCODE_MODE;
		}
		else if(act_mode == GIMBAL_CENTER){
			motor_mode = GYRO_MODE;
		}
		else if(act_mode == SELF_GYRO){
			motor_mode = GYRO_MODE;
		}
	}
#else
	BoardMode_t    board_mode = AUTO_AIM_MODE;
	BoardActMode_t act_mode   = GIMBAL_FOLLOW;
	GimbalMotorMode_t motor_mode = ENCODE_MODE;
	rc_hdlr->control_mode = PC_MODE;
#endif

	/* set modes */
	gimbal_set_mode(gbal, board_mode);
	gimbal_set_act_mode(gbal, act_mode);// act mode only works when debuging with rc
	gimbal_set_motor_mode(gbal, motor_mode);
}

/******************************** For Comms Above **************************************/
/******************************** For Auto Aiming Below ********************************/
static void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, RemoteControl_t *rc_hdlr, UC_Auto_Aim_Pack_t *pack){
	float cur_yaw_target = 0.0;
	float cur_pitch_target = 0.0;
	float delta_yaw= 0.0;
	float delta_pitch = 0.0;

	if(rc_hdlr->control_mode == PC_MODE){
		/* filter applied here, TODO may add kalman filter here, depends on data input */
		float filtered_delta_yaw = ewma_filter(&gbal->ewma_f_aim_yaw, pack->delta_yaw);
//		pack->yaw_data = sliding_window_mean_filter(&gbal->swm_f_aim_yaw, pack->yaw_data);
		delta_yaw = in_out_map(filtered_delta_yaw, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI

		float filtered_delta_pitch = ewma_filter(&gbal->ewma_f_aim_pitch, pack->delta_pitch);
//		pack->pitch_data = sliding_window_mean_filter(&gbal->swm_f_aim_pitch, pack->yaw_data);
		delta_pitch = in_out_map(filtered_delta_pitch, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI
	}
	/* get the latest angle position of pitch and yaw motor */
	gimbal_get_ecd_fb_data(&gimbal,
						   &(motor_data[yaw_id].motor_feedback),
						   &(motor_data[pitch_id].motor_feedback));
	/* NOTE: Even if the target was beyond pi, the motor still tracked the same dir bc of spd loop and phase delay,
	 * and right about next time, the feedback of motor would be changed from pi to -pi(or inverse), which will
	 * also update the target into right scale of angle */
	if(gbal->gimbal_motor_mode == GYRO_MODE){
		cur_yaw_target = gbal->yaw_cur_abs_angle - delta_yaw; // only yaw use abs values
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch;
	}
	else{
		cur_yaw_target = gbal->yaw_cur_rel_angle - delta_yaw;
		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch;
	}
	/* avoid small noise to spin the yaw */
	if(fabs(delta_yaw)>= 1.0f*DEGREE2RAD)
		gbal->yaw_tar_angle = cur_yaw_target;
	if(fabs(delta_pitch)>= 1.0f*DEGREE2RAD)
		gbal->pitch_tar_angle = cur_pitch_target;
	/* independent mode don't allow set yaw angle */
	if(gbal->gimbal_act_mode == INDPET_MODE)
		gbal->yaw_tar_angle = 0;
}
/******************************** For Auto Aiming Above ********************************/
#endif

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void gimbal_cmd_exec(Gimbal_t *gbal, uint8_t mode){
	if(mode == DUAL_LOOP_PID_CONTROL)
	 /* set motor voltage through cascade pid controller */
		  set_motor_can_volt(gimbal.yaw_tar_angle,
							 gimbal.pitch_tar_angle,
							 0, //
							 0, //set 0 to magazine motor,dual loop control will not affect magazine
							 mode,
							 gimbal.gimbal_motor_mode);
	else if(mode == SINGLE_LOOP_PID_CONTROL){ // only spd control
		motor_data[pitch_id].tx_data = pid_single_loop_control(gbal->pitch_tar_spd,
														&(motor_data[pitch_id].motor_info.s_pid),
														motor_data[pitch_id].motor_feedback.rx_rpm);
		motor_data[yaw_id].tx_data = pid_single_loop_control(gbal->yaw_tar_spd,
														&(motor_data[yaw_id].motor_info.s_pid),
														motor_data[yaw_id].motor_feedback.rx_rpm);
	}
	else{
		motor_data[pitch_id].tx_data = 0;
		motor_data[yaw_id].tx_data = 0;
	}
}

#endif /* __GIMBAL_APP_C__ */
