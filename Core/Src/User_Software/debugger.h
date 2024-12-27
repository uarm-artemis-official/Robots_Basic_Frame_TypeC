/*
 * Debugger.h
 *
 *  Created on Jul, 2023
 *  	Author : Haoran
 *
 * */

void motor_step_signal_generator(float idle_time_100ms, float value, int motor_id);
void motor_spid_step_signal_generator(float idle_time_100ms, float target, float cur, int motor_id, PID_t *s_pid);
void motor_fpid_step_signal_generator(float idle_time_100ms, float target, int motor_id, GimbalMotorMode_t mode);
