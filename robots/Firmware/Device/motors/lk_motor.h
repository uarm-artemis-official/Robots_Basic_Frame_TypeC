#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "motor_types.h"

void lk_motor_send(uint32_t id, LK_Motor_Command_t control_cmd,
                   int32_t sendValue);
void lk_motor_get_raw_feedback(const uint8_t rx_buffer[8], void* fb_ptr);
void lk_motor_send_multi_loop(uint32_t id, uint16_t max_speed, uint32_t angle);
void lk_motor_send_single_loop(uint32_t id, uint8_t spin_direction,
                               uint16_t max_speed, uint32_t angle);
void lk_motor_send_multi_torque_current(int16_t q1, int16_t q2, int16_t q3,
                                        int16_t q4);
void lk_motor_send_single_torque(uint8_t id, int16_t torque_current);
void lk_motor_send_speed(uint8_t id, int32_t speed);
#ifdef __cplusplus
}
#endif

#endif