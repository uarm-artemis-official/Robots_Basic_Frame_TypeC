#include "motors.h"
#include "dji_motor.h"
#include "lk_motor.h"
#include "subsystems_defines.h"
#include "uarm_lib.h"

Motors::Motors() {
    memset(this->motors, 0, sizeof(Generic_Motor_t) * MAX_MOTOR_COUNT);
    this->config = MOTORS_NONE;
}

void Motors::init(Motor_Config_t config) {
    this->config = config;
    memset(this->motors, 0, sizeof(Generic_Motor_t) * MAX_MOTOR_COUNT);

    switch (config) {
        case DJI_GIMBAL: {
            this->motors[0] = Generic_Motor_t {SHOOT_LEFT_FRIC, 0, DJI};
            this->motors[1] = Generic_Motor_t {SHOOT_RIGHT_FRIC, 0, DJI};
            this->motors[2] = Generic_Motor_t {GIMBAL_YAW, 0, DJI};
            this->motors[3] = Generic_Motor_t {GIMBAL_PITCH, 0, DJI};
            this->motors[4] = Generic_Motor_t {SHOOT_LOADER, 0, DJI};
            break;
        }
        case DJI_CHASSIS: {
            this->motors[0] = Generic_Motor_t {CHASSIS_WHEEL1, 0, DJI};
            this->motors[1] = Generic_Motor_t {CHASSIS_WHEEL2, 0, DJI};
            this->motors[2] = Generic_Motor_t {CHASSIS_WHEEL3, 0, DJI};
            this->motors[3] = Generic_Motor_t {CHASSIS_WHEEL4, 0, DJI};
            break;
        }
        case SWERVE: {
            // TODO
            break;
        }
        default:
            ASSERT(0,
                   "Attempt to configure subsystems::motors to unknown "
                   "configuration.");
    }
}

void Motors::parse_feedback(uint32_t stdid, uint8_t data[8],
                            Motor_Feedback_t* feedback) {
    if (0x200 < stdid && stdid < 0x212) {
        dji_motor_parse_feedback(data, feedback);
    } else if (0 < stdid && stdid < 0x140) {  // TODO: Change to valid limits
        lk_motor_parse_feedback(data, feedback);
    } else {
        ASSERT(0,
               "subsystems::motors cannot parse feedback for data with "
               "unsupported stdid");
    }
}

void Motors::set_motor_voltage(Motor_CAN_ID_t can_id, int32_t output) {
    for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (this->motors[i].feedback_id == can_id) {
            this->motors[i].tx_data = output;
            break;
        }
    }
}

void Motors::send_motor_voltage() {
    switch (this->config) {
        case DJI_GIMBAL:
            dji_motor_send((int32_t) GM6020, this->motors[2].tx_data,
                           this->motors[3].tx_data, this->motors[4].tx_data, 0);
            break;
        case DJI_CHASSIS:
            dji_motor_send((int32_t) M3508, this->motors[0].tx_data,
                           this->motors[1].tx_data, this->motors[2].tx_data,
                           this->motors[3].tx_data);
            break;
        case SWERVE:
            // TODO
            break;
        default:
            ASSERT(0, "Attempt to send for an unknown motors configuration.");
    }
}
