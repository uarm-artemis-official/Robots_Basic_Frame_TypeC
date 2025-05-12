#ifndef __DJI_MOTOR_H
#define __DJI_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_types.h"
#include "can.h"

void dji_motor_parse_feedback(const uint8_t *can_data, Motor_Feedback_t *motor_feedback);
void dji_motor_send(int32_t id, int32_t d1, int32_t d2, int32_t d3, int32_t d4);

#ifdef __cplusplus
}
#endif

#endif