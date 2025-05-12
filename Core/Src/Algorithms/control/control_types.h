#ifndef __CONTROL_TYPES_H
#define __CONTROL_TYPES_H

#include "uarm_types.h"

/* =========================================================================
 * FEED FORWARD TYPES
 * ====================================================================== */
/**
  * @brief  feedforward main structure
  * @Note
  */
typedef struct{
	float ff_gain;
	float last_input;
	float output;
}FeedForward_t;


/* =========================================================================
 * PID TYPES
 * ====================================================================== */

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


/* =========================================================================
 * RAMP TYPES
 * ====================================================================== */

typedef struct ramp_t
{
  int32_t count;
  int32_t scale;
  float   out;
  void  (*init)(struct ramp_t *ramp, int32_t scale);
  float (*calc)(struct ramp_t *ramp);
}ramp_t;


#endif