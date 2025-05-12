#ifndef __MOTORS_H
#define __MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems_types.h"

void motors_subsystem_init(System_Motors_t *system, Motor_Config_t config);
void parse_feedback(uint32_t stdid, uint8_t data[8], Motor_Feedback_t *feedback);
void set_motor_voltage(System_Motors_t *system, Motor_CAN_ID_t can_id, int32_t output);
void send_motor_voltage(System_Motors_t *system);

#ifdef __cplusplus
}
#endif

#endif