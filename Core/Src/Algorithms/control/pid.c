/*
 * pid.c
 *
 *  Created on: Jul, 2023
 *      Author: Haoran
 */
#include "pid.h"
#include "maths.h"

/**
  * @brief  pid parameters initialization
  * @retval None
  */
void pid_param_init(PID_t *pid, int32_t max_out, float max_i_out, float max_err, float kp, float ki, float kd){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->max_out = max_out;
	pid->max_i_out = max_i_out;
	pid->max_err = max_err;

	pid->err = 0;
	pid->last_err = 0;
	pid->llast_err = 0;

	pid->total_out = 0;

}

/**
  * @brief     calculate positional PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float pid_calculate(PID_t *pid, float cur_val, float target_val)
{
  float dt = 1.0f; //sampling time
  pid->cur_val = cur_val;
  pid->target_val = target_val;
  pid->last_err = pid->err;
  pid->err =  target_val - cur_val;
  if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
    return 0;
  pid->pout = pid->kp * pid->err;
  pid->iout += pid->ki * pid->err *dt;
  pid->dout = pid->kd * (pid->err - pid->last_err) / dt;

  abs_limit(&(pid->iout), pid->max_i_out);
  pid->total_out = pid->pout + pid->iout + pid->dout;
  abs_limit(&(pid->total_out), pid->max_out);

  return pid->total_out;
}

/**
  * @brief     calculate incremental PID
  * @param[in] pid: pid control struct
  * @param[in] cur_val: current value
  * @param[in] target_val: target value
  * @retval    pid calculate output
  */
float pid_incremental_calculate(PID_t *pid, float cur_val, float target_val)
{
  float dt = 1.0f; //sampling time
  pid->cur_val = cur_val;
  pid->target_val = target_val;
  pid->llast_err = pid->last_err;
  pid->last_err = pid->err;
  pid->err = target_val - cur_val;
  if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
    return 0;

  pid->pout = pid->kp * (pid->err - pid->last_err);
  pid->iout += pid->ki *dt;
  pid->dout = pid->kd * (pid->err - 2.0f*pid->last_err + pid->llast_err) / dt;

  abs_limit(&(pid->iout), pid->max_i_out);
  pid->total_out = pid->pout + pid->iout + pid->dout;
  abs_limit(&(pid->total_out), pid->max_out);

  /* pid dead zone risk management */
  if(fabs(pid->total_out) < 10)
	  pid->total_out = 0;

  return pid->total_out;
}
/**
  * @brief  Single-loop pid controller
  * @retval None
  */
float pid_single_loop_control(float target_val, PID_t *pid, float cur_val){
	return pid_calculate(pid, cur_val, target_val);
}

/**
  * @brief  Dual-loop pid controller
  * @Note 	Dual control provide more force and greater torque
  * @retval None
  */
float pid_dual_loop_control(float f_tar_val, PID_t *f_pid, PID_t *s_pid, float f_cur_val, float s_cur_val){
	float f_out=0;
	f_out = pid_calculate(f_pid, f_cur_val, f_tar_val);
	return pid_calculate(s_pid, s_cur_val, f_out);
}

