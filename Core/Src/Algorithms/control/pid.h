/**
 * There are different forms of PID controllers depending on how the 
 * governing equations of the controller are defined. Here are some:
 * 
 * (Parallel form) 
 * u(t) = K_p * e(t) + K_i * int^t_0 e(\tau) d\tau + K_d * de(t)/dt
 * 
 * There are individual gains per PID segment (i.e. there is one 
 * proportional gain, one integral gain, and one derivative gain).
 * This allows for tuning of each individual segment independently
 * from one another. Critism of this form is that gains are not tied
 * to any physical meaning and are just numbers that have to be toggled.
 * 
 * (Standard form)
 * u(t) = K_p * [e(t) + 1/T_i \int^t_0 e(\tau) d\tau + T_d * de(t)/dt]
 * 
 * Commonly used form in industry where K_p term is factored out and
 * applied to all segments of the controller. Gains in this form are 
 * considered more grounded in physical reality with K_p considered a 
 * scaling term, T_i is integral time (how long to eliminate past errors),
 * T_d is derivative time (how far into the future should be considered to 
 * predict error value). Cohen-Coon and Lambda tuning methods were developed
 * for this form.
 * 
 * (Series form)
 * An older form used in pneumatic and electronic controllers. 
 * Ziegler-Nichols tuning method was developed for this form.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "control_types.hpp"

// TODO Convert to structs or classes.
//declare pid functions
void pid_param_init(PID_t* pid, int32_t max_out, float max_i_out, float max_err,
                    float kp, float ki, float kd);
float pid_calculate(PID_t* pid, float get, float set, float dt);
float pid_single_loop_control(float target_val, PID_t* pid, float cur_val,
                              float dt);
float pid_dual_loop_control(float f_tar_val, PID_t* f_pid, PID_t* s_pid,
                            float f_cur_val, float s_cur_val, float dt);

void pid2_init(PID2_t& pid, float k_p, float k_i, float k_d, float beta,
               float yeta, float min_out, float max_out);
void pid2_set_limits(PID2_t& pid, float new_min_out, float new_max_out);
float pid2_calculate(PID2_t& pid, float sp, float pv, float dt);
float pid2_single_loop_control(PID2_t& pid, float sp, float pv, float dt);
float pid2_dual_loop_control(PID2_t& f_pid, PID2_t& s_pid, float sp, float f_pv,
                             float s_pv, float f_dt, float s_dt);
float pid2_triple_loop_control(PID2_t& outer_pid, PID2_t& middle_pid,
                               PID2_t& inner_pid, float sp, float outer_pv,
                               float middle_pv, float inner_pv, float outer_dt,
                               float middle_dt, float inner_dt);

void prescaled_pid2_init(Prescaled_PID2_t* prescaled, uint32_t prescalar,
                         float k_p, float k_i, float k_d, float beta,
                         float yeta, float min_out, float max_out);

void prescaled_pid2_single_loop_control(Prescaled_PID2_t* prescaled, float sp,
                                        float pv, float dt);

#endif