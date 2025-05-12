/*******************************************************************************
* @file           : Shoot_App.h
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SHOOT_APP_H__
#define __SHOOT_APP_H__

// #include "string.h"
// #include "tim.h"
// #include "public_defines.h"
// #include "buzzer.h"
// #include "motor.h"
// #include "ramp.h"
// #include "message_center.h"
// #include "apps_types.h"

// extern TIM_HandleTypeDef htim1;
// extern TIM_HandleTypeDef htim2;
// extern TIM_HandleTypeDef htim4;
// extern TIM_HandleTypeDef htim8;

/* functions declaration here */
void Shoot_Task_Func(void const * argument);
// void shoot_task_init(Shoot_t *sht, Motor_t *s_motors);
// void shoot_firc_init(Shoot_t *sht);
// void shoot_params_init(Shoot_t *sht);
// void shoot_servo_init(void);
// void shoot_fric_pwm_engagement(Shoot_t *sht, uint16_t target_pwm);
// void shoot_fric_can_engagement(Shoot_t *sht, Motor_t *s_motors, uint16_t target_can);
// void set_shoot_mode(Shoot_t *sht, ShootActMode_t mode);
// void set_lid_status(Shoot_t *sht, ShootLidStatus_t status);
// void set_mag_motor_speed(Shoot_t *sht, float spd);
// void set_mag_motor_angle(Shoot_t *sht, float tar_angle);
// void set_fric_motor_speed(Shoot_t *sht, int16_t spd);
// void set_fric_motor_current(Shoot_t *sht, int16_t spd);
// void set_servo_value(uint16_t pwm_value);
// void shoot_get_motor_feedback(Shoot_t *shoot, Motor_t *s_motors);
// void shoot_calc_loader_pid_out(Shoot_t *sht, Motor_t *s_motors);
// void shoot_calc_fric_pid_out(Motor_t *motor, float target);
// void shoot_send_motor_volts(Motor_t *s_motors);
// void shoot_mag_get_rel_angle(Shoot_t *sht);
// void shoot_detect_mag_status(Shoot_t *sht);
// void shoot_stop(Shoot_t *sht);
// void shoot_execute(Shoot_t *sht, Motor_t *s_motors);
// void shoot_get_rc_info(Shoot_t *shoot);
// int16_t shoot_mag_update_turns(Shoot_t *sht, int16_t raw_ecd, int16_t prev_ecd);



#endif /* __SHOOT_APP_H__ */
