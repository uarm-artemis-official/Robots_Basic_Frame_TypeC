#include "motors.h"
#include "dji_motor.h"
#include "lk_motor.h"
#include "subsystems_defines.h"
#include "uarm_lib.h"

void motors_subsystem_init(System_Motors_t* system, Motor_Config_t config) {
    memset(system, 0, sizeof(System_Motors_t));

    system->config = config;
    switch (config) {
        case DJI_GIMBAL: {
            system->motors[0] = Generic_Motor_t {SHOOT_LEFT_FRIC, 0, DJI};
            system->motors[1] = Generic_Motor_t {SHOOT_RIGHT_FRIC, 0, DJI};
            system->motors[2] = Generic_Motor_t {GIMBAL_YAW, 0, DJI};
            system->motors[3] = Generic_Motor_t {GIMBAL_PITCH, 0, DJI};
            system->motors[4] = Generic_Motor_t {SHOOT_LEFT_FRIC, 0, DJI};
            break;
        }
        case DJI_CHASSIS: {
            system->motors[0] = Generic_Motor_t {CHASSIS_WHEEL1, 0, DJI};
            system->motors[1] = Generic_Motor_t {CHASSIS_WHEEL2, 0, DJI};
            system->motors[2] = Generic_Motor_t {CHASSIS_WHEEL3, 0, DJI};
            system->motors[3] = Generic_Motor_t {CHASSIS_WHEEL4, 0, DJI};
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

void parse_feedback(uint32_t stdid, uint8_t data[8],
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

void set_motor_voltage(System_Motors_t* system, Motor_CAN_ID_t can_id,
                       int32_t output) {
    for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (system->motors[i].feedback_id == can_id) {
            system->motors[i].tx_data = output;
            break;
        }
    }
}

void send_motor_voltage(System_Motors_t* system) {
    switch (system->config) {
        case DJI_GIMBAL:
            dji_motor_send((int32_t) GM6020, system->motors[2].tx_data,
                           system->motors[3].tx_data, system->motors[4].tx_data,
                           0);
            break;
        case DJI_CHASSIS:
            dji_motor_send((int32_t) M3508, system->motors[0].tx_data,
                           system->motors[1].tx_data, system->motors[2].tx_data,
                           system->motors[3].tx_data);
            break;
        case SWERVE:
            // TODO
            break;
        default:
            ASSERT(0, "Attempt to send for an unknown motors configuration.");
    }
}