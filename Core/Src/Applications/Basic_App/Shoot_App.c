/*******************************************************************************
* @file           : Shoot_App.c
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SHOOT_APP_C__
#define __SHOOT_APP_C__


#include "Shoot_App.h"


//#define SHOOT_HEAT_LIMIT 1

/* sentry only */
int16_t shoot_counter= 0;
uint8_t shoot_reserve_flag = 0;
uint8_t shoot_reserve_counter = 0;
uint8_t shoot_check_flag = 0;
uint16_t shoot_check_counter = 0;

static Shoot_t shoot;
static Motor_t shoot_motors[3];

//FIXME: Once we have referee system, we can limit the motor power
void Shoot_Task_Func(void const * argument)
{
  /* Infinite loop */
  shoot_task_init(&shoot, shoot_motors);

  /* set task exec period */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SHOOT_TASK_EXEC_TIME); //

  /* init the task ticks */
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {
	  shoot_get_rc_info(&shoot);

	  /* select lid status */
//	  shoot_lid_status_selection(&shoot, &rc);

	  /* get feedback of the magazine motor */
	  shoot_get_motor_feedback(&shoot, shoot_motors);
	  shoot_mag_get_rel_angle(&shoot);

	  /* check the magazine status */
//	  shoot_detect_mag_status(&shoot);

	 /* determine if open lid */
	 if(shoot.lid_status == OPEN){//if sentry, delete this function
		set_servo_value(SERVO_PWM_OPEN_LID);
	 }
	 else if(shoot.lid_status == CLOSE){
		set_servo_value(SERVO_PWM_CLOSE_LID);
	 }

	 /* formal shoot task functions begins */
	 if(shoot.shoot_act_mode == SHOOT_CEASE){
		shoot.mag_turns_counter = 0;//clear magazine turns
		shoot_stop(&shoot);
//	  		buzzer_stop();
	 }
	 else if(shoot.shoot_act_mode == SHOOT_RESERVE){
		  /* reserve the magazine motor for a while */
		  //FIXME: didn't consider if the reserve spin also stuck

		  /* Reset timer13 flag and counter */
		  shoot_reserve_flag = 0;
		  shoot_reserve_counter = 0;

		  /* Set backward magazine and friction motor */
		  set_mag_motor_angle(&shoot, -0.6*PI); //
		  set_fric_motor_current(&shoot, -LEVEL_ONE_CAN_SPD * 0.5);

	  }
	  else if(shoot.shoot_act_mode == SHOOT_ONCE){
		 /* need referee system to determine shooting spd */
		  set_mag_motor_angle(&shoot, 0.33*PI);
		  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
	  }
	  else if(shoot.shoot_act_mode == SHOOT_TRIPLE){
		  /* need referee system to determine shooting spd */
	 	  set_mag_motor_angle(&shoot, PI);
	 	  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
	  }
	  else if(shoot.shoot_act_mode == SHOOT_CONT){
//		  if(rc.pc.mouse.right_click.status == PRESSED){
//			  /* auto aimming engage */
//			  /* Well I guess we don't need this any more since we should control the shooting
//			   * at any time during competition due to shooting heat */
//		  }
		  /* FIXME need referee system to determine shooting spd */
		  set_mag_motor_angle(&shoot, shoot.mag_cur_angle + SHOOT_CONT_MAG_SPEED);//keep spinning
		  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
	  }
	  shoot_execute(&shoot, shoot_motors);
	  shoot_send_motor_volts(shoot_motors);

	  /* delay until wake time */
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief     shoot task initialization
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_task_init(Shoot_t *sht, Motor_t *s_motors) {
	/* init pid of magazine motor */
	// Note this is only for 2006, the pid params need to fine tune with the actual payload
	motor_init(&(s_motors[SHOOT_LOADER_2006_INDEX]), max_out_angle_mag_2006,  max_I_out_angle_mag_2006, max_err_angle_mag_2006, //angular loop
								kp_angle_mag_2006, ki_angle_mag_2006, kd_angle_mag_2006,
							max_out_spd_mag_2006, max_I_out_spd_mag_2006, max_err_spd_mag_2006, //spd loop
								kp_spd_mag_2006, ki_spd_mag_2006, kd_spd_mag_2006,
							kf_spd_mag_2006);//spd ff gain
	motor_init(&(s_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX]), max_out_spd_fric,  max_I_out_spd_fric, max_err_spd_fric, kp_spd_fric, ki_spd_fric, kd_spd_fric,
								 0, 0, 0, 0, 0, 0,//no second loop
								 0);//spd ff gain
	motor_init(&(s_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX]), max_out_spd_fric,  max_I_out_spd_fric, max_err_spd_fric, kp_spd_fric, ki_spd_fric, kd_spd_fric,
							 0, 0, 0, 0, 0, 0,//no second loop
							 0);//spd ff gain

	ramp_init(&shoot.fric_left_ramp, FRIC_CAN_RAMP_DELAY);
	ramp_init(&shoot.fric_right_ramp, FRIC_CAN_RAMP_DELAY);

	/* init servo motor */
	shoot_servo_init();

	/* init parameters */
	shoot_params_init(sht);

	/* reset feedback value */
	memset(&(s_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].motor_feedback), 0, sizeof(Motor_Feedback_t));
	memset(&(s_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].motor_feedback), 0, sizeof(Motor_Feedback_t));
	memset(&(s_motors[SHOOT_LOADER_2006_INDEX].motor_feedback), 0, sizeof(Motor_Feedback_t));
	memset(&(sht->mag_fb), 0, sizeof(Motor_Feedback_t));

	/* set shoot mode */
	set_shoot_mode(sht, SHOOT_CEASE);
	set_lid_status(sht, CLOSE);
	set_fric_motor_current(sht, 0);
}

void shoot_params_init(Shoot_t *sht){
	sht->mag_cur_angle = 0;
	sht->mag_tar_angle = 0;
	sht->mag_pre_ecd_angle = 0;
	sht->mag_tar_spd   = 0;
	sht->fric_can_tar_spd = 0;
	sht->mag_turns_counter = 0;
	sht->mag_center_offset = 0;
	sht->prev_angle_reset = 1;
	sht->fric_engage_flag = 0;
	sht->fric_left_cur_spd = 0;
	sht->fric_right_cur_spd = 0;
	sht->fric_counter = 0;
	sht->lid_counter = 0;
}

void shoot_servo_init(void){
	/* Start PWM */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);

	/* adjust to zero degree */
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_PWM_CLOSE_LID);
}

/**
  * @brief     set the shoot action mode
  * @param[in] main struct of shoot task
  * @param[in] mode: shoot act mode
  * @retval    None
  */
void set_shoot_mode(Shoot_t *sht, ShootActMode_t mode){
	sht->shoot_act_mode = mode;
}

void set_lid_status(Shoot_t *sht, ShootLidStatus_t status){
	sht->lid_status = status;
}

void set_mag_motor_speed(Shoot_t *sht, float spd){
	sht->mag_tar_spd = spd;
}


void set_mag_motor_angle(Shoot_t *sht, float tar_angle){//-pi, pi
	if(sht->shoot_act_mode == SHOOT_ONCE){
		/* for once, the input target is a rel angle of current shaft */
		sht->mag_tar_angle = tar_angle;
	}
	else{
		/* for burst shooting, just set the input target */
		sht->mag_tar_angle = tar_angle; // (-pi, pi)
		sht->mag_turns_counter = 0;
	}
}

void set_fric_motor_speed(Shoot_t *sht, int16_t spd){
	sht->fric_tar_spd = spd;
}

void set_fric_motor_current(Shoot_t *sht, int16_t spd){
	sht->fric_can_tar_spd = spd;
}

void set_servo_value(uint16_t pwm_value){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}
/**
  * @brief     shoot cease fire
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_stop(Shoot_t *sht){

	/* stop magazine motor first */
	set_mag_motor_angle(sht, 0);
	sht->mag_cur_angle = 0;//ensure the err is 0

	/* stop friction wheel*/
	/* really depends on the type of fric motors used */
#ifndef USE_CAN_FRIC
	sht->fric_tar_spd = 1100;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sht->fric_tar_spd);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sht->fric_tar_spd);
	/* reset each value */

	sht->fric_left_ramp.count = 0;
	sht->fric_right_ramp.count = 0;
	sht->fric_engage_flag = 0;
	sht->fric_left_cur_spd = 0;
	sht->fric_right_cur_spd = 0;
#else
	set_fric_motor_current(sht, 0);
	sht->fric_engage_flag = 0;
	sht->fric_left_ramp.count = 0;
	sht->fric_right_ramp.count = 0;
	sht->fric_left_cur_spd = 0;
	sht->fric_right_cur_spd = 0;
	sht->fric_counter = 0;
#endif
}

/**
  * @brief     friction engage functions, for pwm-based motor
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_fric_pwm_engagement(Shoot_t *sht, uint16_t target_pwm){
	/* snail speed controller need to be engaged one by one */
	if(sht->fric_engage_flag == 0){
		if(sht->fric_left_cur_spd >= target_pwm){
			/* engage right fric wheel */
			sht->fric_right_cur_spd = ramp_calculate(&shoot.fric_right_ramp) * target_pwm + target_pwm;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sht->fric_right_cur_spd);
			if(sht->fric_right_cur_spd >= target_pwm){
				sht->fric_engage_flag = 1;
				osDelay(500);
			}
			return;
		}
		/* engage left wheel first, use ramp function to graduately enhance the pwm value */
		sht->fric_left_cur_spd = ramp_calculate(&shoot.fric_left_ramp) * target_pwm + target_pwm;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sht->fric_left_cur_spd);
	}
}

/**
  * @brief     friction engage functions, for can-based motor
  * @param[in] shoot main struct
  * @param[in] target can torque current -> rpm / speed
  * @retval    None
  */
void shoot_fric_can_engagement(Shoot_t *sht, Motor_t *s_motors, uint16_t target_can) {
	/* obtain motor feedback for determining the current rpm */
//	shoot_fric_get_feedback(sht);
	sht->fric_left_cur_spd = s_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].motor_feedback.rx_rpm;
	sht->fric_right_cur_spd = s_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].motor_feedback.rx_rpm;

	if(sht->fric_engage_flag == 0){
		/* engage fric wheel using ramp funcion */
//		  sht->fric_can_tar_spd  = ramp_calculate(&shoot.fric_left_ramp)  * target_can;
//		  sht->fric_can_tar_spd  = ramp_calculate(&shoot.fric_right_ramp) * target_can;
		/* engage fric wheel without ramp funcion */
		  sht->fric_can_tar_spd = target_can;
		  sht->fric_counter++; // delay counter, when counter reaches given value then engage mag
		  	  	  	  	  	   // delay time = fric_counter * ABStaskdelay
		}
	else{
		sht->fric_can_tar_spd = target_can;
	}
	/* update send value of CAN */
	// Motor_t *s_motors, int16_t motor_index, int16_t can_id, float target
	shoot_calc_fric_pid_out(&(shoot_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX]), -sht->fric_can_tar_spd);
	shoot_calc_fric_pid_out(&(shoot_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX]), sht->fric_can_tar_spd);

//	shoot_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].tx_data = 250;
//	shoot_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].tx_data = -250;
}

/**
  * @brief     shoot main execute function
  * 			call this to engage fire process
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_execute(Shoot_t *sht, Motor_t *s_motors) {
	/* try single loop first, not considering single shoot using angle loop */
	if(sht->shoot_act_mode == SHOOT_CEASE || sht->shoot_act_mode == SHOOT_RESERVE) {
//		shoot_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].tx_data = 0;
//		shoot_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].tx_data = 0;
		shoot_calc_fric_pid_out(&(shoot_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX]), sht->fric_can_tar_spd);
		shoot_calc_fric_pid_out(&(shoot_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX]), -sht->fric_can_tar_spd);
	} else {
		shoot_fric_can_engagement(sht, s_motors, sht->fric_can_tar_spd);//
		if(sht->fric_engage_flag == 0 && sht->fric_counter >=FRIC_CAN_RAMP_DELAY) {
//			osDelay(500);
			sht->fric_engage_flag = 1;
			sht->fric_counter = 0;
		}
	}
	/* activate magazine later */
	if(sht->shoot_act_mode == SHOOT_CEASE || sht->shoot_act_mode == SHOOT_RESERVE)
		shoot_calc_loader_pid_out(sht, s_motors);
	else if(sht->fric_engage_flag == 1) // frictions are engaged
		shoot_calc_loader_pid_out(sht, s_motors);
}


void shoot_get_motor_feedback(Shoot_t *shoot, Motor_t *s_motors) {
	Motor_Feedback_t motor_feedbacks[8];
	BaseType_t motor_feedback_message = peek_message(MOTOR_READ, motor_feedbacks, 0);
	if (motor_feedback_message == pdTRUE) {
		memcpy(&(shoot->mag_fb), &motor_feedbacks[SHOOT_LOADER_CAN_ID], sizeof(Motor_Feedback_t));
		memcpy(&(s_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].motor_feedback), &motor_feedbacks[SHOOT_LEFT_FRIC_CAN_ID], sizeof(Motor_Feedback_t));
		memcpy(&(s_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].motor_feedback), &motor_feedbacks[SHOOT_RIGHT_FRIC_CAN_ID], sizeof(Motor_Feedback_t));
		memcpy(&(s_motors[SHOOT_LOADER_2006_INDEX].motor_feedback), &motor_feedbacks[SHOOT_LOADER_CAN_ID], sizeof(Motor_Feedback_t));
	}
}

/**
  * @brief     shoot mode selection based on
  * @param[in] shoot main struct
  * @retval    None
  */
//void shoot_fric_get_feedback(Shoot_t *sht){
//	Motor_Feedback_t motor_feedbacks[8];
//	BaseType_t motor_feedback_message = peek_message(MOTOR_READ, motor_feedbacks, 0);
//	if (motor_feedback_message == pdTRUE) {
//		memcpy(&(sht->left_fric_fb), &motor_feedbacks[fric_left_id], sizeof(Motor_Feedback_t));
//		memcpy(&(sht->right_fric_fb), &motor_feedbacks[fric_right_id], sizeof(Motor_Feedback_t));
//	}
//}
//
///* Magazine Angular&Speed PID Control */
///**
//  * @brief     shoot mode selection based on
//  * @param[in] shoot main struct
//  * @retval    None
//  */
//void shoot_mag_get_feedback(Shoot_t *sht){
////	memcpy(&(sht->mag_fb), &motor_data[mag_2006_id].motor_feedback, sizeof(Motor_Feedback_t));
//}

void shoot_mag_get_rel_angle(Shoot_t *sht){
	/* get latest feedback of mag motor */
//	shoot_mag_get_feedback(sht);
	/* update truns */
	shoot_mag_update_turns(sht, sht->mag_fb.rx_angle, sht->mag_pre_ecd_angle);
	/* calca current mag angle, range is roughly (0, 2pi)*/
	sht->mag_cur_angle = (sht->mag_turns_counter*2*PI / SHOOT_MAG_GEAR_RATIO) + // the angle the turns has been done
						 (sht->mag_fb.rx_angle/8192*(2*PI)/SHOOT_MAG_GEAR_RATIO);// the current rx angle
	/* mapped from encoder value to (-pi, pi) */
	sht->mag_pre_ecd_angle = sht->mag_fb.rx_angle;
	/* update turns */
}

/*
 * @brief     Get latest turns of magazine motor from previous ecd angle.
 * @param[in] raw_ecd: abs yaw ecd angle from feedback
 * @param[in] prev_ecd: the center offset of ecd mode
 * */
int16_t shoot_mag_update_turns(Shoot_t *sht, int16_t raw_ecd, int16_t prev_ecd)
{
	//FiXME: this 4096 value actually depends on sampling time of the fedback
	//		 we now assume that the motor would not spin beyond half a cycle between
	//	     two samples.(depends on the rpm and task ticks, use uart to output)
    if (raw_ecd - prev_ecd < -4096)//fine tuning here
    	sht->mag_turns_counter++;
    else if (raw_ecd - prev_ecd > 4096)
        sht->mag_turns_counter--;
    return 0;
}


void shoot_calc_loader_pid_out(Shoot_t *sht, Motor_t *s_motors) {
	s_motors[SHOOT_LOADER_2006_INDEX].tx_data = (int32_t) pid_dual_loop_control(sht->mag_tar_angle,
																				 &(s_motors[SHOOT_LOADER_2006_INDEX].motor_info.f_pid),
																				 &(s_motors[SHOOT_LOADER_2006_INDEX].motor_info.s_pid),
																				 sht->mag_cur_angle,
																				 sht->mag_fb.rx_rpm,
																				 SHOOT_TASK_EXEC_TIME * 0.001);
}


void shoot_calc_fric_pid_out(Motor_t *motor, float target) {
	motor->tx_data = pid_single_loop_control(target,
											&(motor->motor_info.f_pid),
											motor->motor_feedback.rx_current,
											SHOOT_TASK_EXEC_TIME*0.001);
}


void shoot_send_motor_volts(Motor_t *s_motors) {
	MotorSetMessage_t motor_set_message;
	motor_set_message.motor_can_volts[SHOOT_LOADER_CAN_ID] = s_motors[SHOOT_LOADER_2006_INDEX].tx_data;
	motor_set_message.motor_can_volts[SHOOT_LEFT_FRIC_CAN_ID] = s_motors[SHOOT_LEFT_FRIC_WHEEL_INDEX].tx_data;
	motor_set_message.motor_can_volts[SHOOT_RIGHT_FRIC_CAN_ID] = s_motors[SHOOT_RIGHT_FRIC_WHEEL_INDEX].tx_data;
	motor_set_message.data_enable = (1 << SHOOT_LOADER_CAN_ID) |
									(1 << SHOOT_LEFT_FRIC_CAN_ID) |
									(1 << SHOOT_RIGHT_FRIC_CAN_ID);
	pub_message(MOTOR_SET, &motor_set_message);
}


void shoot_get_rc_info(Shoot_t *shoot) {
	RCInfoMessage_t rc_info;
	BaseType_t new_rc_info_message = peek_message(RC_INFO, &rc_info, 0);
	if (new_rc_info_message == pdTRUE) {
		ShootActMode_t shoot_mode = rc_info.modes[2];
		set_shoot_mode(shoot, shoot_mode);
	}
}


/**
  * @brief     shoot mode selection based on input rc switch
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
//static void shoot_mode_rc_selection(Shoot_t *sht, RemoteControl_t *rc){
//	ShootActMode_t mode;
//	if(rc->control_mode == CTRLER_MODE){
//		/* always judge cease fire first */
//		if (rc->ctrl.s2 == SW_UP && rc->ctrl.s1 != SW_MID) {
//			mode = SHOOT_CONT;
//		} else {
//			mode = SHOOT_CEASE;
//		}
//		set_shoot_mode(sht, mode);
//	}
//	else if(rc->control_mode == PC_MODE){
//		/* always judge cease fire first */
//		if(rc->pc.mouse.left_click.status == RELEASED){
//			mode = SHOOT_CEASE;
//			rc->pc.mouse.left_click.pre_status = RELEASED;
//		}
//		else{
//			if(rc->pc.mouse.left_click.status == PRESSED){
//				mode = SHOOT_CONT;
//				rc->pc.mouse.left_click.pre_status = PRESSED;
//				if(rc->pc.key.B.status == PRESSED){
//					mode = SHOOT_RESERVE;
//					rc->pc.key.B.status = PRESSED;
//				}
//			}
//		}
//		set_shoot_mode(sht, mode);
//	}
//}

/**
  * @brief     determine if we need to open/close lid
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
//static void shoot_lid_status_selection(Shoot_t *sht, RemoteControl_t *rc){
//	if(rc->pc.key.R.status == RELEASED_TO_PRESS && rc->pc.key.R.pre_status != RELEASED_TO_PRESS){
//		sht->lid_counter++;
//		rc->pc.key.R.pre_status = RELEASED_TO_PRESS;
//	}
//	else
//		rc->pc.key.R.pre_status = rc->pc.key.R.status;
//	if(sht->lid_counter == 1)
//		set_lid_status(sht, OPEN);
//	else if(sht->lid_counter == 2){
//		set_lid_status(sht, CLOSE);
//		sht->lid_counter = 0;
//	}
//}
/**
  * @brief     check if we need to reserve the mag motor
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
void shoot_detect_mag_status(Shoot_t *sht){
	if(sht->shoot_act_mode != SHOOT_CEASE){
		/* check if the magazine motor stuck */
		if(abs(sht->mag_fb.rx_rpm)<=10 && sht->mag_fb.rx_temp > 0)
			/* engage check process */
			shoot_check_flag = 1;
		else{
			/* if not, clear flag and counter*/
			shoot_check_flag = 0;
			shoot_check_counter = 0;
		}

		/* if the flag has been set and count more than 1s */
		//FIXME: Need to test the actual check duration
		if(shoot_check_flag != 1 || (shoot_check_flag == 1 && shoot_check_counter < 10) )
			return;// motor process normally or check time less than 2s
		else{//shoot_check_counter >= 20
			/* set auto reserve process */
			set_shoot_mode(sht, SHOOT_RESERVE);
			/* clear flags and counter */
			shoot_check_flag = 0;
			shoot_check_counter = 0;
		}
	}
}

#ifdef SHOOT_HEAT_LIMIT
/* deploy shoot heat managment here based on the referee system */
#endif




#endif /* __SHOOT_APP_C__*/

