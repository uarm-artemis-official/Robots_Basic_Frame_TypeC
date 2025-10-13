/**
 * @file pid.cpp
 * @brief Implementation of all control PID systems.
 * @details All implementations for control systems that utilizes PID as the primary controller
 * are included in this file. This includes single and cascade control loops as well as
 * different implementations of PID with their own solutions to problems known about
 * PID controllers in real systems (e.g. integator windup, derivative kick, etc.).
 * @author Haoran Qi, and James Fu 
 * @date 2025-07-17
 */

#include "pid.h"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

void pid2_init(PID2_t& pid, float k_p, float k_i, float k_d, float beta,
               float yeta, float min_out, float max_out) {
    ASSERT(max_out >= min_out, "max_out has to be greater or equal to min_out");

    pid.k_p = k_p;
    pid.k_i = k_i;
    pid.k_d = k_d;
    pid.beta = beta;
    pid.yeta = yeta;

    pid.plant_value = 0;
    pid.setpoint = 0;

    pid.p_error = 0;
    pid.i_error = 0;
    pid.d_error = 0;
    pid.prev_d_error = 0;

    pid.p_out = 0;
    pid.i_out = 0;
    pid.d_out = 0;

    pid.max_out = max_out;
    pid.min_out = min_out;

    pid.prev_total_out = 0;
    pid.total_out = 0;
}

void pid2_set_limits(PID2_t& pid, float new_min_out, float new_max_out) {
    ASSERT(new_max_out >= new_min_out,
           "new_max_out has to be greater or equal to new_min_out.");

    pid.max_out = new_max_out;
    pid.min_out = new_min_out;

    // TODO: Implement anti-integrator windup
}

float pid2_calculate(PID2_t& pid, float sp, float pv, float dt) {
    ASSERT(dt > 0, "PID must have a timestep.");
    pid.plant_value = pv;
    pid.setpoint = sp;
    pid.prev_d_error = pid.d_error;
    pid.prev_total_out = pid.total_out;

    // Calculate error terms.
    pid.p_error = pid.beta * sp - pv;
    pid.i_error = sp - pv;
    pid.d_error = pid.yeta * sp - pv;

    pid.p_out = pid.k_p * pid.p_error;

    // Anti-integrator windup
    if (pid.min_out < pid.prev_total_out && pid.prev_total_out < pid.max_out) {
        pid.i_out += pid.k_i * pid.i_error * dt;
    }

    pid.d_out = pid.k_d * (pid.d_error - pid.prev_d_error) / dt;
    pid.total_out = pid.p_out + pid.i_out + pid.d_out;

    pid.total_out = value_limit(pid.total_out, pid.min_out, pid.max_out);
    return pid.total_out;
}

float pid2_single_loop_control(PID2_t& pid, float sp, float pv, float dt) {
    return pid2_calculate(pid, sp, pv, dt);
}

float pid2_dual_loop_control(PID2_t& f_pid, PID2_t& s_pid, float sp, float f_pv,
                             float s_pv, float f_dt, float s_dt) {
    float f_out = pid2_calculate(f_pid, sp, f_pv, f_dt);
    return pid2_calculate(s_pid, f_out, s_pv, s_dt);
}

float pid2_triple_loop_control(PID2_t& outer_pid, PID2_t& middle_pid,
                               PID2_t& inner_pid, float sp, float outer_pv,
                               float middle_pv, float inner_pv, float outer_dt,
                               float middle_dt, float inner_dt) {
    float outer_out = pid2_calculate(outer_pid, sp, outer_pv, outer_dt);
    float middle_out =
        pid2_calculate(middle_pid, outer_out, middle_pv, middle_dt);
    return pid2_calculate(inner_pid, middle_out, inner_pv, inner_dt);
}

void prescaled_pid2_init(Prescaled_PID2_t* prescaled, uint32_t prescalar,
                         float k_p, float k_i, float k_d, float beta,
                         float yeta, float min_out, float max_out) {
    prescaled->prescalar = prescalar;
    prescaled->prescalar_count = 0;
    prescaled->cumsum_dt = 0;

    pid2_init(prescaled->pid, k_p, k_i, k_d, beta, yeta, min_out, max_out);
}

void prescaled_pid2_single_loop_control(Prescaled_PID2_t* prescaled, float sp,
                                        float pv, float dt) {
    prescaled->prescalar_count++;
    prescaled->cumsum_dt += dt;
    if (prescaled->prescalar_count >= prescaled->prescalar) {
        pid2_calculate(prescaled->pid, sp, pv, prescaled->cumsum_dt);
        prescaled->prescalar_count = 0;
        prescaled->cumsum_dt = 0;
    }
}
