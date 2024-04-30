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
#include "tim.h"
#include "public_defines.h"
#include "string.h"
#include "stdio.h"

//#define SHOOT_HEAT_LIMIT 1

/* static functions declares here */
static void shoot_mode_rc_selection(Shoot_t *sht, RemoteControl_t *rc);
static void shoot_lid_status_selection(Shoot_t *sht, RemoteControl_t *rc);

/* sentry only */
int16_t shoot_counter= 0;
extern TIM_HandleTypeDef htim1;

//FIXME: Once we have referee system, we can limit the motor power
void Shoot_Task_Func(void const * argument)
{
  /* Infinite loop */
  shoot_task_init(&shoot);

  /* set task exec period */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // task exec period 10ms

  /* init the task ticks */
  xLastWakeTime = xTaskGetTickCount();

  for(;;)
  {

	  //FIXME: rc debug needed
//	  if(gimbal.gimbal_mode == DEBUG_MODE)
	  shoot_mode_rc_selection(&shoot, &rc);

	  /* select lid status */
	  shoot_lid_status_selection(&shoot, &rc);

	  /* get feedback of the magazine motor */
	  shoot_mag_get_rel_angle(&shoot);

	  /* check the magazine status */
	  shoot_detect_mag_status(&shoot);

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
		  shoot_reserve_flag = 1;
		  while(shoot_reserve_counter<20){//20*100ms = 2s
			  set_mag_motor_angle(&shoot, shoot.mag_cur_angle - 0.3*PI);
#ifndef USE_CAN_FRIC
			  set_fric_motor_speed(&shoot, LEVEL_ONE_PWM);
#else
			  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
#endif
			  shoot_execute(&shoot);
			  vTaskDelayUntil(&xLastWakeTime, xFrequency);
		  }
		  /* reset timer13 flag and counter */
		  shoot_reserve_flag = 0;
		  shoot_reserve_counter = 0;
	  }
	  else if(shoot.shoot_act_mode == SHOOT_ONCE){
		 /* need referee system to determine shooting spd */
		  set_mag_motor_angle(&shoot, 0.3*PI);
#ifndef USE_CAN_FRIC
		  set_fric_motor_speed(&shoot, LEVEL_ONE_PWM);
#else
		  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
#endif
	  }
	  else if(shoot.shoot_act_mode == SHOOT_CONT){
		  if(rc.pc.mouse.right_click.status == PRESSED){
			  /* auto aimming engage */
			  /* Well I guess we don't need this any more since we should control the shooting
			   * at any time during competition due to shooting heat */
		  }
		  /* FIXME need referee system to determine shooting spd */
		  set_mag_motor_angle(&shoot, shoot.mag_cur_angle + SHOOT_CONT_MAG_SPEED);//keep spinning
#ifndef USE_CAN_FRIC
		  set_fric_motor_speed(&shoot, LEVEL_ONE_PWM);
#else
		  set_fric_motor_current(&shoot, LEVEL_ONE_CAN_SPD);
#endif
	  }
	  shoot_execute(&shoot);

	  /* delay until wake time */
	  vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief     shoot task initialization
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_task_init(Shoot_t *sht){
	/* init pid of magazine motor */
	// Note this is only for 2006, the pid params need to fine tune with the actual payload
	motor_init(mag_2006_id, max_out_angle_mag_2006,  max_I_out_angle_mag_2006, max_err_angle_mag_2006, //angular loop
								kp_angle_mag_2006, ki_angle_mag_2006, kd_angle_mag_2006,
							max_out_spd_mag_2006, max_I_out_spd_mag_2006, max_err_spd_mag_2006, //spd loop
								kp_spd_mag_2006, ki_spd_mag_2006, kd_spd_mag_2006,
							kf_spd_mag_2006);//spd ff gain
	// This is for hero's mag, delete above param init when deploy this on Hero and tune pid params.
//	motor_init(mag_3508_id, max_out_angle_mag_3508,  max_I_out_angle_mag_3508, max_err_angle_mag_3508, //angular loop
//								kp_angle_mag_3508, ki_angle_mag_3508, kd_angle_mag_3508,
//							max_out_spd_mag_3508, max_I_out_spd_mag_3508, max_err_spd_mag_3508, //spd loop
//								kp_spd_mag_3508, ki_spd_mag_3508, kd_spd_mag_3508,
//							kf_spd_mag_3508);//spd ff gain

	/* init friction motors */
	shoot_firc_init(&shoot);
#ifndef USE_CAN_FRIC
	ramp_init(&shoot.fric_left_ramp, (LEVEL_ONE_PWM-MIN_PWM_ON_TIME));
	ramp_init(&shoot.fric_right_ramp, (LEVEL_ONE_PWM-MIN_PWM_ON_TIME));
#else
	ramp_init(&shoot.fric_left_ramp, FRIC_CAN_RAMP_DELAY);
	ramp_init(&shoot.fric_right_ramp, FRIC_CAN_RAMP_DELAY);
#endif

	/* init servo motor */
	shoot_servo_init();

	/* init parameters */
	shoot_params_init(sht);

	/* reset feedback value */
	memset(&(sht->mag_fb), 0, sizeof(Motor_Feedback_Data_t));
	memset(&(sht->left_fric_fb), 0, sizeof(Motor_Feedback_Data_t));
	memset(&(sht->right_fric_fb), 0, sizeof(Motor_Feedback_Data_t));

	/* set shoot mode */
	set_shoot_mode(sht, SHOOT_CEASE);
	set_lid_status(sht, STOP);

	/* set comm packs init target number */
	vision_message.message.vision.target_num = 0;
}

/**
  * @brief     friction wheel motor init, depends on motors type
  * @param[in] None
  * @retval    None
  */
void shoot_firc_init(Shoot_t *sht){
#ifndef USE_CAN_FRIC
//	Corresponds to pin H and pin F on the PWM board - need to change manually if we change the pins
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
//	osDelay(100);
//	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//	//osDelay(2000);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MAX_PWM_ON_TIME);
//	osDelay(100);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MAX_PWM_ON_TIME);
//	osDelay(2000);
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MIN_PWM_ON_TIME);
//	osDelay(100);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_PWM_ON_TIME);
//	osDelay(1750);

	/* or snail motor */
	osDelay(3000);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MIN_PWM_ON_TIME);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MIN_PWM_ON_TIME);
	osDelay(3000);
#else
	/* the pid params need to fine tune with the actual payload */
	motor_init(fric_left_id, max_out_spd_fric,  max_I_out_spd_fric, max_err_spd_fric, kp_spd_fric, ki_spd_fric, kd_spd_fric,
							 0, 0, 0, 0, 0, 0,//no second loop
							 0);//spd ff gain
	motor_init(fric_right_id, max_out_spd_fric,  max_I_out_spd_fric, max_err_spd_fric, kp_spd_fric, ki_spd_fric, kd_spd_fric,
							 0, 0, 0, 0, 0, 0,//no second loop
							 0);//spd ff gain
	set_fric_motor_current(sht, 0);
#endif
}

void shoot_params_init(Shoot_t *sht){
	sht->mag_cur_angle = 0;
	sht->mag_tar_angle = 0;
	sht->mag_pre_ecd_angle = 0;
	sht->mag_tar_spd   = 0;
#ifndef USE_CAN_FRIC
	sht->fric_tar_spd = MIN_PWM_ON_TIME;
#else
	sht->fric_can_tar_spd = 0;
#endif
	sht->mag_turns_counter = 0;
	sht->mag_center_offset = 0;
	sht->prev_angle_reset = 1;
	sht->fric_engage_flag = 0;
	sht->fric_left_cur_spd = 0;
	sht->fric_right_cur_spd = 0;
	sht->fric_counter = 0;
}

void shoot_servo_init(void){
	/* Start PWM */
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);

	/* adjust to zero degree */
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, SERVO_PWM_STOP_LID);
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
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_value);
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
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MIN_PWM_ON_TIME);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_PWM_ON_TIME);
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
void shoot_fric_can_engagement(Shoot_t *sht, uint16_t target_can){
	/* obtain motor feedback for determining the current rpm */
	shoot_fric_get_feedback(sht);
	sht->fric_left_cur_spd = sht->left_fric_fb.rx_rpm;
	sht->fric_right_cur_spd = sht->right_fric_fb.rx_rpm;

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
	set_motor_can_current(-sht->fric_can_tar_spd, // left  fric
						  sht->fric_can_tar_spd,// right fric
						  0,
						  0,
						  SINGLE_LOOP_PID_CONTROL);

}

/**
  * @brief     shoot main execute function
  * 			call this to engage fire process
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_execute(Shoot_t *sht){
	/* then activate fric wheels motor */
#ifndef USE_CAN_FRIC
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, sht->fric_tar_spd);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, sht->fric_tar_spd);
	if(sht->shoot_act_mode == SHOOT_CEASE || sht->shoot_act_mode == SHOOT_RESERVE){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, sht->fric_tar_spd);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, sht->fric_tar_spd);
	}
    else
    	shoot_fric_pwm_engagement(sht, sht->fric_tar_spd);
#else
	/* try single loop first, not considering single shoot using angle loop */
	if(sht->shoot_act_mode == SHOOT_CEASE || sht->shoot_act_mode == SHOOT_RESERVE){
		set_motor_can_current(sht->fric_can_tar_spd, // left  fric
							  -sht->fric_can_tar_spd,// right fric
							  0,
							  0,
							  SINGLE_LOOP_PID_CONTROL);
	}else{
		shoot_fric_can_engagement(sht, sht->fric_can_tar_spd);//
		if(sht->fric_engage_flag == 0 && sht->fric_counter >=FRIC_CAN_RAMP_DELAY){
//			osDelay(500);
			sht->fric_engage_flag = 1;
		}
	}
#endif
	/* activate magazine later */
	if(sht->shoot_act_mode == SHOOT_CEASE || sht->shoot_act_mode == SHOOT_RESERVE)
		shoot_mag_dual_loop_control(&shoot);
	else if(sht->fric_engage_flag == 1) // frictions are engaged
		shoot_mag_dual_loop_control(&shoot);
}

/**
  * @brief     shoot mode selection based on
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_fric_get_feedback(Shoot_t *sht){
	memcpy(&(sht->left_fric_fb), &motor_data[fric_left_id].motor_feedback, sizeof(Motor_Feedback_Data_t));
	memcpy(&(sht->right_fric_fb), &motor_data[fric_right_id].motor_feedback, sizeof(Motor_Feedback_Data_t));

}

/* Magazine Angular&Speed PID Control */
/**
  * @brief     shoot mode selection based on
  * @param[in] shoot main struct
  * @retval    None
  */
void shoot_mag_get_feedback(Shoot_t *sht){
	memcpy(&(sht->mag_fb), &motor_data[mag_2006_id].motor_feedback, sizeof(Motor_Feedback_Data_t));
}

void shoot_mag_get_rel_angle(Shoot_t *sht){
	/* get latest feedback of mag motor */
	shoot_mag_get_feedback(sht);
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

/*
 * @brief     angle/spd dual control of magazine motor
 * @param[in] main shoot struct
 * @param[in] prev_ecd: the center offset of ecd mode
 * */
void shoot_mag_dual_loop_control(Shoot_t *sht){
	/* This is only for 2006 motor, used for infantry and sentry */
	motor_data[mag_2006_id].tx_data = pid_dual_loop_control(sht->mag_tar_angle,
												 &(motor_data[mag_2006_id].motor_info.f_pid),
												 &(motor_data[mag_2006_id].motor_info.s_pid),
												 sht->mag_cur_angle,
												 sht->mag_fb.rx_rpm);//pid without ff
	/* this is for 3508, used for hero */
//	motor_data[mag_3508_id].tx_data = pid_dual_loop_control(sht->mag_tar_angle,
//												 &(motor_data[mag_3508_id].motor_info.f_pid),
//												 &(motor_data[mag_3508_id].motor_info.s_pid),
//												 sht->mag_cur_angle,
//												 sht->mag_fb.rx_rpm);
}

/**
  * @brief     shoot mode selection based on input rc switch
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
static void shoot_mode_rc_selection(Shoot_t *sht, RemoteControl_t *rc){
	ShootActMode_t mode;
	if(rc->control_mode == CTRLER_MODE){
		/* always judge cease fire first */
		if (rc->ctrl.s2 == SW_UP && rc->ctrl.s1 != SW_MID) {
			mode = SHOOT_CONT;
		} else {
			mode = SHOOT_CEASE;
		}
		set_shoot_mode(sht, mode);
	}
	else if(rc->control_mode == PC_MODE){
		/* always judge cease fire first */
		if(rc->pc.mouse.left_click.status == RELEASED){
			mode = SHOOT_CEASE;
			rc->pc.mouse.left_click.pre_status = RELEASED;
		}
		else{
			if(rc->pc.mouse.left_click.status == PRESSED){
				mode = SHOOT_CONT;//SHOOT_CONT;
				rc->pc.mouse.left_click.pre_status = PRESSED;
			}
			else if(rc->pc.mouse.left_click.status == RELEASED_TO_PRESS){//check rising edge
				mode = SHOOT_ONCE;//SHOOT_CONT;
			}

		}
		set_shoot_mode(sht, mode);
	}
}

/**
  * @brief     determine if we need to open/close lid
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
static void shoot_lid_status_selection(Shoot_t *sht, RemoteControl_t *rc){
	if(rc->pc.key.C.status == RELEASED_TO_PRESS){
		if(sht->lid_status != OPEN){
			set_lid_status(sht, OPEN);
		}
		else if(sht->lid_status == OPEN){
			set_lid_status(sht, CLOSE);
		}
	}
}
/**
  * @brief     check if we need to reserve the mag motor
  * @param[in] shoot main struct
  * @param[in] rc main struct
  * @retval    None
  */
void shoot_detect_mag_status(Shoot_t *sht){
	if(sht->shoot_act_mode != SHOOT_CEASE){
		/* check if the magazine motor stuck */
		if(abs(sht->mag_fb.rx_rpm)<=10)
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

