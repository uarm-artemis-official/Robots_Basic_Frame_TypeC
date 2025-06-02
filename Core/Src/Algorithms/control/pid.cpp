/*
 * pid.c
 *
 *  Created on: Jul, 2023
 *      Author: Haoran
 */
#include "pid.h"
#include "uarm_lib.h"
#include "uarm_math.h"

/**
  * @brief  pid parameters initialization
  * @retval None
  */
void pid_param_init(PID_t* pid, int32_t max_out, float max_i_out, float max_err,
                    float kp, float ki, float kd) {
    ASSERT(max_out >= 0, "max_out has to be non-negative.");
    ASSERT(max_err >= 0, "max_err has to be non-negative.");
    ASSERT(max_i_out >= 0, "max_i_out has to be non-negative.");

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->cur_val = 0.f;
    pid->target_val = 0.f;

    pid->err = 0.f;
    pid->last_err = 0.f;
    pid->llast_err = 0.f;

    pid->pout = 0.f;
    pid->iout = 0.f;
    pid->dout = 0.f;
    pid->max_out = max_out;
    pid->max_err = max_err;
    pid->max_i_out = max_i_out;
    pid->total_out = 0.f;
}

/**
  * @brief     calculate positional PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output
  */
float pid_calculate(PID_t* pid, float cur_val, float target_val, float dt) {
    dt = 1.0f;  //sampling time
    pid->cur_val = cur_val;
    pid->target_val = target_val;
    pid->last_err = pid->err;
    pid->err = target_val - cur_val;
    if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
        return 0;
    pid->pout = pid->kp * pid->err;
    pid->iout += pid->ki * pid->err * dt;
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
float pid_incremental_calculate(PID_t* pid, float cur_val, float target_val) {
    float dt = 1.0f;  //sampling time
    pid->cur_val = cur_val;
    pid->target_val = target_val;
    pid->llast_err = pid->last_err;
    pid->last_err = pid->err;
    pid->err = target_val - cur_val;
    if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
        return 0;

    pid->pout = pid->kp * (pid->err - pid->last_err);
    pid->iout += pid->ki * dt;
    pid->dout =
        pid->kd * (pid->err - 2.0f * pid->last_err + pid->llast_err) / dt;

    abs_limit(&(pid->iout), pid->max_i_out);
    pid->total_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->total_out), pid->max_out);

    /* pid dead zone risk management */
    if (fabs(pid->total_out) < 10)
        pid->total_out = 0;

    return pid->total_out;
}
/**
  * @brief  Single-loop pid controller
  * @retval None
  */
float pid_single_loop_control(float target_val, PID_t* pid, float cur_val,
                              float dt) {
    return pid_calculate(pid, cur_val, target_val, dt);
}

/**
  * @brief  Dual-loop pid controller
  * @Note 	Dual control provide more force and greater torque
  * @retval None
  */
float pid_dual_loop_control(float f_tar_val, PID_t* f_pid, PID_t* s_pid,
                            float f_cur_val, float s_cur_val, float dt) {
    float f_out = 0;
    f_out = pid_calculate(f_pid, f_cur_val, f_tar_val, dt);
    return pid_calculate(s_pid, s_cur_val, f_out, dt);
}

void pid2_init(PID2_t* pid, float k_p, float k_i, float k_d, float beta,
               float yeta, float min_out, float max_out) {
    ASSERT(max_out >= min_out, "max_out has to be greater or equal to min_out");

    pid->k_p = k_p;
    pid->k_i = k_i;
    pid->k_d = k_d;
    pid->beta = beta;
    pid->yeta = yeta;

    pid->plant_value = 0;
    pid->setpoint = 0;

    pid->p_error = 0;
    pid->i_error = 0;
    pid->d_error = 0;
    pid->prev_d_error = 0;

    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;

    pid->max_out = max_out;
    pid->min_out = min_out;

    pid->prev_total_out = 0;
    pid->total_out = 0;
}

float pid2_calculate(PID2_t* pid, float sp, float pv, float dt) {
    pid->plant_value = pv;
    pid->setpoint = sp;
    pid->prev_d_error = pid->d_error;
    pid->prev_total_out = pid->total_out;

    // Calculate error terms.
    pid->p_error = pid->beta * sp - pv;
    pid->i_error = sp - pv;
    pid->d_error = pid->yeta * sp - pv;

    pid->p_out = pid->k_p * pid->p_error;

    // Anti-integrator wind up
    if (pid->min_out < pid->prev_total_out &&
        pid->prev_total_out < pid->max_out) {
        pid->i_out += pid->k_i * pid->i_error * dt;
    }

    pid->d_out = pid->k_d * (pid->d_error - pid->prev_d_error) / dt;
    pid->total_out = pid->p_out + pid->i_out + pid->d_out;

    if (pid->total_out > pid->max_out)
        pid->total_out = pid->max_out;
    if (pid->total_out < pid->min_out)
        pid->total_out = pid->min_out;
    return pid->total_out;
}

float pid2_single_loop_control(PID2_t* pid, float sp, float pv, float dt) {
    return pid2_calculate(pid, sp, pv, dt);
}

float pid2_dual_loop_control(PID2_t* f_pid, PID2_t* s_pid, float sp, float f_pv,
                             float s_pv, float f_dt, float s_dt) {
    float f_out = pid2_calculate(f_pid, sp, f_pv, f_dt);
    return pid2_calculate(s_pid, f_out, s_pv, s_dt);
}

void prescaled_pid2_init(Prescaled_PID2_t* prescaled, uint32_t prescalar,
                         float k_p, float k_i, float k_d, float beta,
                         float yeta, float min_out, float max_out) {
    prescaled->prescalar = prescalar;
    prescaled->prescalar_count = 0;
    prescaled->cumsum_dt = 0;

    pid2_init(&(prescaled->pid), k_p, k_i, k_d, beta, yeta, min_out, max_out);
}

void prescaled_pid2_single_loop_control(Prescaled_PID2_t* prescaled, float sp,
                                        float pv, float dt) {
    prescaled->prescalar_count++;
    prescaled->cumsum_dt += dt;
    if (prescaled->prescalar_count >= prescaled->prescalar) {
        pid2_calculate(&(prescaled->pid), sp, pv, prescaled->cumsum_dt);
        prescaled->prescalar_count = 0;
        prescaled->cumsum_dt = 0;
    }
}
