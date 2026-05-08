#include <cstring>
#include "dji_motor_driver.hpp"
#include "lk_motor_driver.hpp"
#include "middleware_classes.hpp"
#include "subsystems_classes.hpp"
#include "subsystems_defines.hpp"
#include "uarm_lib.hpp"

namespace {
    // TODO: Remove
    constexpr MW_CAN::BUS k_motor_bus = MW_CAN::BUS::CAN_1;

    bool send_motor_frame(MW_CAN::ICAN& can, const MW_CAN::CANFrame& frame) {
        uint8_t payload[8] = {0};
        for (size_t i = 0; i < frame.dlc; ++i) {
            payload[i] = std::to_integer<uint8_t>(frame.payload[i]);
        }

        return can.send_data(k_motor_bus, frame.sid, frame.eid, payload,
                             frame.dlc);
    }

    MW_CAN::CANFrame build_can_frame(uint32_t stdid, const uint8_t data[8]) {
        MW_CAN::CANFrame frame;
        frame.sid = stdid;
        frame.eid = 0;
        frame.is_extended_id = false;
        frame.dlc = 8;
        for (size_t i = 0; i < frame.dlc; ++i) {
            frame.payload[i] = std::byte {data[i]};
        }
        return frame;
    }
}  // namespace

Motors::Motors(MW_CAN::ICAN& can_ref) : can(can_ref) {
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
            this->motors[2] = Generic_Motor_t {SHOOT_TOP_FRIC, 0, DJI};
            this->motors[3] = Generic_Motor_t {GIMBAL_YAW, 0, DJI};
            this->motors[4] = Generic_Motor_t {GIMBAL_PITCH, 0, DJI};
            this->motors[5] = Generic_Motor_t {SHOOT_LOADER, 0, DJI};
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
        case MOTORS_NONE:
            break;
        default:
            ASSERT(0,
                   "Attempt to configure subsystems::motors to unknown "
                   "configuration.");
    }
}

void Motors::get_raw_feedback(uint32_t stdid, uint8_t data[8], void* feedback) {
    MW_CAN::CANFrame frame = build_can_frame(stdid, data);

    // TODO: Make better limits.
    if (0x200 < stdid && stdid < 0x212) {
        dji_motor::MotorFeedback parsed_feedback;
        ASSERT(dji_motor::parse_feedback_message(parsed_feedback, frame),
               "Failed to parse DJI motor feedback message.");

        Motor_Feedback_t* raw_feedback =
            static_cast<Motor_Feedback_t*>(feedback);
        raw_feedback->rx_angle = parsed_feedback.encoder_angle;
        raw_feedback->rx_rpm = parsed_feedback.rpm;
        raw_feedback->rx_current = parsed_feedback.current;
        raw_feedback->rx_temp = parsed_feedback.temperature;
    } else if (0x140 < stdid && stdid < 0x173) {
        if (data[0] ==
            static_cast<uint8_t>(lk_motor::MotorCommand::READ_ENCODER_FB)) {
            LK_Motor_Torque_Feedback_t* lk_feedback =
                static_cast<LK_Motor_Torque_Feedback_t*>(feedback);
            ASSERT(lk_motor::parse_encoder_feedback(lk_feedback->ecd_position,
                                                    frame),
                   "Failed to parse LK encoder feedback message.");
        } else if (data[0] ==
                   static_cast<uint8_t>(
                       lk_motor::MotorCommand::SL_ANGLE_WITH_SPEED)) {
            lk_motor::TorqueFeedback parsed_feedback;
            ASSERT(lk_motor::parse_torque_feedback(parsed_feedback, frame),
                   "Failed to parse LK torque feedback message.");

            LK_Motor_Torque_Feedback_t* lk_feedback =
                static_cast<LK_Motor_Torque_Feedback_t*>(feedback);
            lk_feedback->temperature = parsed_feedback.temperature;
            lk_feedback->torque_current = parsed_feedback.torque_current;
            lk_feedback->speed = parsed_feedback.speed;
            lk_feedback->ecd_position = parsed_feedback.ecd_position;
        } else {
            ASSERT(false, "Trying to parse unsupported feedback.");
        }
    } else {
        ASSERT(false,
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

void Motors::set_motor_voltage(uint32_t can_id, int32_t output) {
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

void Motors::send_motor_voltage() {
    MW_CAN::CANFrame message;

    switch (this->config) {
        case DJI_GIMBAL:
            // Gimbal and loader.
            dji_motor::format_voltage_message(
                dji_motor::MotorType::GM6020, this->motors[3].tx_data,
                this->motors[4].tx_data, this->motors[5].tx_data, 0, message);
            (void) send_motor_frame(can, message);

            // Flywheels
            dji_motor::format_voltage_message(
                dji_motor::MotorType::GM3510, this->motors[0].tx_data,
                this->motors[1].tx_data, this->motors[2].tx_data, 0, message);
            (void) send_motor_frame(can, message);
            break;
        case DJI_CHASSIS:
            dji_motor::format_voltage_message(
                dji_motor::MotorType::GM3510, this->motors[0].tx_data,
                this->motors[1].tx_data, this->motors[2].tx_data,
                this->motors[3].tx_data, message);
            (void) send_motor_frame(can, message);
            break;
        case SWERVE: {
            dji_motor::format_voltage_message(
                dji_motor::MotorType::GM3510, motors[0].tx_data,
                motors[1].tx_data, motors[2].tx_data, motors[3].tx_data,
                message);
            (void) send_motor_frame(can, message);

            // TODO: Add encoding + decoding library for swerve in lib.
            uint8_t spin_direction =
                (motors[4 + counter].tx_data & 0x40000000) >> 30;
            uint16_t max_speed =
                (motors[4 + counter].tx_data & 0x3fff0000) >> 16;
            uint32_t new_angle = (motors[4 + counter].tx_data & 0xffff) * 10;
            if (new_angle == 0) {
                new_angle = 1;
            }

            lk_motor::format_single_loop_message(
                0x141 + counter, spin_direction, max_speed, new_angle, message);
            (void) send_motor_frame(can, message);

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
    MW_CAN::CANFrame message;

    switch (config) {
        case SWERVE_ZERO:
        case SWERVE:
            if (Motors::get_motor_brand(can_id) == LK) {
                lk_motor::format_control_message(
                    can_id, lk_motor::MotorCommand::READ_SL_FB, 0, message);
                (void) send_motor_frame(can, message);
            }
            break;
        default:
            return;
    }
}
