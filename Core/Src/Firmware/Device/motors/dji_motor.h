#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "can.h"
#include "motor_types.h"

void dji_motor_get_raw_feedback(const uint8_t* can_data,
                                Motor_Feedback_t* motor_feedback);
void __dji_motor_send(int32_t id, int32_t d1, int32_t d2, int32_t d3,
                      int32_t d4, uint8_t use_can2);
void dji_motor_send_voltage(int32_t id, int32_t d1, int32_t d2, int32_t d3,
                            int32_t d4);
void dji_motor_send_current(int32_t i1, int32_t i2, int32_t i3, int32_t i4);

#ifdef __cplusplus
}
#endif

#endif