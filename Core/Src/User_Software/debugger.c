/*
 * Integrate all the debugger task for tuning the devices
 *
 * Created on Jul, 2023
 * 		Author: Haoran
 *
 * */

#ifndef __DEV_DEBUGGER_C__
#define __DEV_DEBUGGER_C__

#include <Gimbal_App.h>
#include "public_defines.h"
#include "stdio.h"


extern Motor motor_data[MOTOR_COUNT];
extern uint32_t debugger_signal_counter;
extern uint32_t debugger_signal_flag;
#ifdef GIMBAL_MOTOR_DEBUG
	extern PID_t tune_pid_f;
	extern PID_t tune_pid_s;
#endif /* GIMBAL_MOTOR_DEBUG */


/**
  * @brief     generate step signal for motor to tune parameters, for raw data
  * @param[in] idle_time_100ms: idle time before generating signals
  * @param[in] value: the target value set
  * @param[in] motor_id: motor can id
  * @retval    None
  */
void motor_step_signal_generator(float idle_time_100ms, float value, int motor_id){
	debugger_signal_flag = 1;
	if(debugger_signal_counter >= idle_time_100ms){
		motor_data[motor_id].tx_data = value;
	}
	else{
		motor_data[motor_id].tx_data = 0;
	}
}

/**
  * @brief     generate step signal for motor to tune parameters, for inner loop
  * @param[in] idle_time_100ms: idle time before generating signals
  * @param[in] value: the target value set
  * @param[in] motor_id: motor can id
  * @retval    None
  */
void motor_spid_step_signal_generator(float idle_time_100ms, float target, float cur, int motor_id, PID_t *s_pid){
	debugger_signal_flag = 1;
	if(debugger_signal_counter >= idle_time_100ms){
		/* for second loop control  parameters */
		motor_data[motor_id].tx_data = pid_single_loop_control(target,
															   s_pid,
															   cur,
															   GIMBAL_TASK_EXEC_TIME*0.001);
	}
	else{
		motor_data[motor_id].tx_data = 0;
	}
}

/**
  * @brief     generate step signal for motor to tune parameters, for outer loop
  * @param[in] idle_time_100ms: idle time before generating signals
  * @param[in] value: the target value set
  * @param[in] motor_id: motor can id
  * @retval    None
  */
void motor_fpid_step_signal_generator(float idle_time_100ms, float target, int motor_id, GimbalMotorMode_t mode){
	debugger_signal_flag = 1;
	if(debugger_signal_counter >= idle_time_100ms){
		if(motor_id == YAW_ID)
		/* for outer loop control  parameters */
			set_motor_debug_can_volt(target, 0, 0, 0, DUAL_LOOP_PID_CONTROL, mode);
		else if(motor_id == PITCH_ID)
			set_motor_debug_can_volt(0, target, 0, 0, DUAL_LOOP_PID_CONTROL, mode);
	}
	else{
		motor_data[motor_id].tx_data = 0;
	}
}





#endif /*__DEV_DEBUGGER_C__*/
