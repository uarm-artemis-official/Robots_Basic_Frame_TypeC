/*
 * pid related funtions headers
 *
 * Jul, 2023
 * 		Author: Haoran
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control_types.h"

//declare pid functions
void pid_param_init(PID_t* pid, int32_t max_out, float max_i_out, float max_err,
                    float kp, float ki, float kd);
float pid_calculate(PID_t* pid, float get, float set, float dt);
float pid_incremental_calculate(PID_t* pid, float cur_val, float target_val);
float pid_single_loop_control(float target_val, PID_t* pid, float cur_val,
                              float dt);
float pid_dual_loop_control(float f_tar_val, PID_t* f_pid, PID_t* s_pid,
                            float f_cur_val, float s_cur_val, float dt);

void pid2_init(PID2_t* pid, float k_p, float k_i, float k_d, float beta,
               float yeta, float min_out, float max_out);
float pid2_calculate(PID2_t* pid, float sp, float pv, float dt);
float pid2_single_loop_control(PID2_t* pid, float sp, float pv, float dt);
float pid2_dual_loop_control(PID2_t* f_pid, PID2_t* s_pid, float sp, float f_pv,
                             float s_pv, float f_dt, float s_dt);

void prescaled_pid2_init(Prescaled_PID2_t* prescaled, uint32_t prescalar,
                         float k_p, float k_i, float k_d, float beta,
                         float yeta, float min_out, float max_out);

void prescaled_pid2_single_loop_control(Prescaled_PID2_t* prescaled, float sp,
                                        float pv, float dt);

#ifdef __cplusplus
}
#endif

#endif
