/*
 * pid related funtions headers
 *
 * Jul, 2023
 * 		Author: Haoran
 *
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"


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
}PID_t;

//declare pid functions
void pid_param_init(PID_t *pid, int32_t max_out, float max_i_out, float max_err, float kp, float ki, float kd);
float pid_calculate(PID_t *pid, float get, float set);
float pid_incremental_calculate(PID_t *pid, float cur_val, float target_val);
float pid_single_loop_control(float target_val, PID_t *pid, float cur_val);
float pid_dual_loop_control(float f_tar_val, PID_t *f_pid, PID_t *s_pid, float f_cur_val, float s_cur_val);



#endif
