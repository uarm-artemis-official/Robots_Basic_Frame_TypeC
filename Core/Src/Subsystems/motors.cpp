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
    memset(prev_swerve_data, 0, sizeof(int32_t) * 4);

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
        case SWERVE_ZERO:
        case SWERVE: {
            this->motors[0] = Generic_Motor_t {CHASSIS_WHEEL1, 0, DJI};
            this->motors[1] = Generic_Motor_t {CHASSIS_WHEEL2, 0, DJI};
            this->motors[2] = Generic_Motor_t {CHASSIS_WHEEL3, 0, DJI};
            this->motors[3] = Generic_Motor_t {CHASSIS_WHEEL4, 0, DJI};
            this->motors[4] = Generic_Motor_t {SWERVE_STEER_MOTOR1, 0, LK};
            this->motors[5] = Generic_Motor_t {SWERVE_STEER_MOTOR2, 0, LK};
            this->motors[6] = Generic_Motor_t {SWERVE_STEER_MOTOR3, 0, LK};
            this->motors[7] = Generic_Motor_t {SWERVE_STEER_MOTOR4, 0, LK};
            break;
        }
        default:
            ASSERT(0,
                   "Attempt to configure subsystems::motors to unknown "
                   "configuration.");
    }
}

void Motors::get_raw_feedback(uint32_t stdid, uint8_t data[8], void* feedback) {
    if (0x200 < stdid && stdid < 0x212) {
        dji_motor_get_raw_feedback(data,
                                   static_cast<Motor_Feedback_t*>(feedback));
    } else if (0x140 < stdid && stdid < 0x173) {
        lk_motor_get_raw_feedback(data, feedback);
    } else {
        ASSERT(0,
               "subsystems::motors cannot parse feedback for data with "
               "unsupported stdid");
    }
}

Motor_Brand_t Motors::get_motor_brand(uint32_t stdid) {
    if (stdid >= 0x141 && stdid <= 0x172) {
        return LK;
    } else if (stdid >= 0x201 && stdid <= 0x208) {
        return DJI;
    } else {
        return UNKNOWN_MOTOR;
    }
}

void Motors::set_motor_voltage(Motor_CAN_ID_t can_id, int32_t output) {
    for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
        if (this->motors[i].feedback_id == can_id) {
            if (is_valid_output(i, output)) {
                this->motors[i].tx_data = output;
            }
            break;
        }
    }
}

bool Motors::is_valid_output(size_t motor_index, int32_t new_output) {
    switch (motors[motor_index].brand) {
        case DJI:
            return -30000 <= new_output && new_output <= 30000;
        case LK:
            return true;
        default:
            return false;
    }
}

static uint32_t counter = 0;
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
        case SWERVE: {
            __dji_motor_send((int32_t) M3508, motors[0].tx_data,
                             motors[1].tx_data, motors[2].tx_data,
                             motors[3].tx_data, 1);

            uint8_t spin_direction =
                (motors[4 + counter].tx_data & 0x40000000) >> 30;
            uint16_t max_speed =
                (motors[4 + counter].tx_data & 0x3fff0000) >> 16;
            uint32_t angle = (motors[4 + counter].tx_data & 0xffff) * 10;
            if (angle == 0) {
                angle = 1;
            }
            lk_motor_send_single_loop(0x141 + counter, spin_direction,
                                      max_speed, angle);
            counter = (counter + 1) % 4;
            break;
        }
        case SWERVE_ZERO:
            break;
        default:
            ASSERT(0, "Attempt to send for an unknown motors configuration.");
    }
}

void Motors::request_feedback(Motor_CAN_ID_t can_id) {
    switch (config) {
        case SWERVE_ZERO:
        case SWERVE:
            if (Motors::get_motor_brand(can_id) == LK)
                lk_motor_send(can_id, LK_MOTOR_READ_SL_FB, 0);
            break;
        default:
            return;
    }
}
