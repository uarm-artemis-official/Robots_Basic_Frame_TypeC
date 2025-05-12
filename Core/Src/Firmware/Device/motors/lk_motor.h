#ifndef __LK_MOTOR_H
#define __LK_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_types.h"
#include "can.h"


void lk_motor_init(uint8_t lk_motor_id);
void lk_motor_send(CAN_HandleTypeDef* hcan, uint32_t id, LK_Motor_Command_t control_cmd, int32_t sendValue);
void lk_motor_parse_feedback(const uint8_t *rx_buffer, Motor_Feedback_t *motor_feedback);

#ifdef __cplusplus
}
#endif

#endif