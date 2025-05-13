#ifndef __MOTORS_H
#define __MOTORS_H

#include "subsystems_defines.h"
#include "subsystems_types.h"

class Motors {
   public:
    Generic_Motor_t motors[MAX_MOTOR_COUNT];
    Motor_Config_t config;

    static void parse_feedback(uint32_t stdid, uint8_t data[8],
                               Motor_Feedback_t* feedback);

    Motors();
    void init(Motor_Config_t config);
    void set_motor_voltage(Motor_CAN_ID_t can_id, int32_t output);
    void send_motor_voltage();
};

#endif