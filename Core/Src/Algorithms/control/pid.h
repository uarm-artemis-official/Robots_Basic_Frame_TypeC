/*
 * pid related funtions headers
 *
 * Jul, 2023
 * 		Author: Haoran
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "maths.h"


#define SINGLE_LOOP_PID_CONTROL 0
#define DUAL_LOOP_PID_CONTROL 1
#define SINGLE_LOOP_SHOOT_CONTROL 2
/*
 * @brief pid structure for all pid control
 * @Note None
 * */
typedef struct {
	float kp;
	float ki;
	float kd;
	float cur_val;
	float target_val;

	float err;
	float last_err;
	float llast_err;

	float pout;
	float iout;
	float dout;
	float max_out;
	float max_err;
	float max_i_out;
	float total_out;
} PID_t;


typedef struct {
	float k_p;
	float k_i;
	float k_d;
	float beta;
	float yeta;

	float plant_value;
	float setpoint;

	float p_error;
	float i_error;
	float d_error;
	float prev_d_error;

	float p_out;
	float i_out;
	float d_out;

	float max_out;
	float min_out;

	float prev_total_out;
	float total_out;
} PID2_t;


//declare pid functions
void pid_param_init(PID_t *pid, int32_t max_out, float max_i_out, float max_err, float kp, float ki, float kd);
float pid_calculate(PID_t *pid, float get, float set, float dt);
float pid_incremental_calculate(PID_t *pid, float cur_val, float target_val);
float pid_single_loop_control(float target_val, PID_t *pid, float cur_val, float dt);
float pid_dual_loop_control(float f_tar_val, PID_t *f_pid, PID_t *s_pid, float f_cur_val, float s_cur_val, float dt);

void pid2_init(PID2_t *pid, float k_p, float k_i, float k_d, float beta, float yeta, float max_out, float min_out);
float pid2_calculate(PID2_t *pid, float sp, float pv, float dt);
float pid2_single_loop_control(PID2_t *pid, float sp, float pv, float dt);
float pid2_dual_loop_control(PID2_t *f_pid, PID2_t *s_pid, float sp, float f_pv, float s_pv, float f_dt, float s_dt);


#endif
