#ifndef __MOTORS_H
#define __MOTORS_H

#include "subsystems_defines.h"
#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class Motors : public IMotors {
   private:
    uint32_t counter = 0;

   public:
    int32_t prev_swerve_data[4];
    Generic_Motor_t motors[MAX_MOTOR_COUNT];
    Motor_Config_t config;

    static void get_raw_feedback(uint32_t stdid, uint8_t data[8],
                                 void* feedback);
    static Motor_Brand_t get_motor_brand(uint32_t stdid);

    Motors();
    void init(Motor_Config_t config) override;
    bool is_valid_output(size_t motor_index, int32_t new_output) override;
    void set_motor_voltage(Motor_CAN_ID_t can_id, int32_t output) override;
    void send_motor_voltage() override;
    void request_feedback(Motor_CAN_ID_t can_id) override;
};

#endif