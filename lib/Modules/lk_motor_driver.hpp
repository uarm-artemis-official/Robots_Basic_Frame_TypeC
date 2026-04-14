#ifndef __LK_MOTOR_HPP
#define __LK_MOTOR_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace lk_motor {
    inline namespace can {
        enum class MotorCommand : uint8_t {
            RESET_ZEROS = 0x19,
            IDLE = 0x80,
            STOP = 0x81,
            START = 0x88,
            READ_ENCODER_FB = 0x90,
            READ_SL_FB = 0x94,
            RESET_POS = 0x95,
            READ_FB_DATA = 0x9C,
            TORQUE_CONTROL = 0xA1,
            ML_ANGLE = 0xA3,
            ML_ANGLE_WITH_SPEED = 0xA4,
            SL_ANGLE_WITH_SPEED1 = 0xA5,
            SL_ANGLE_WITH_SPEED = 0xA6,
            INCREMENTAL_ANGLE = 0xA7,
        };

        struct MotorFeedback {
            int16_t angle;
            int16_t rpm;
            int16_t current;
            int16_t temperature;
        };

        struct TorqueFeedback {
            int8_t temperature;
            int16_t torque_current;
            int16_t speed;
            uint16_t ecd_position;
        };

        void format_control_message(uint32_t id, MotorCommand control_cmd,
                                    int32_t send_value,
                                    MW_CAN::CANFrame& message) {
            const uint32_t send_value_u32 = static_cast<uint32_t>(send_value);

            message.sid = id;
            message.eid = 0U;
            message.is_extended_id = false;
            message.dlc = 8U;

            message.payload[0] =
                static_cast<std::byte>(static_cast<uint8_t>(control_cmd));
            message.payload[1] = std::byte {0};
            message.payload[2] = std::byte {0};
            message.payload[3] = std::byte {0};
            message.payload[4] = static_cast<std::byte>(send_value_u32 & 0xFFU);
            message.payload[5] =
                static_cast<std::byte>((send_value_u32 >> 8) & 0xFFU);
            message.payload[6] =
                static_cast<std::byte>((send_value_u32 >> 16) & 0xFFU);
            message.payload[7] =
                static_cast<std::byte>((send_value_u32 >> 24) & 0xFFU);
        }

        void format_multi_loop_message(uint32_t id, uint16_t max_speed,
                                       uint32_t angle,
                                       MW_CAN::CANFrame& message) {
            message.sid = id;
            message.eid = 0U;
            message.is_extended_id = false;
            message.dlc = 8U;

            message.payload[0] = static_cast<std::byte>(
                static_cast<uint8_t>(MotorCommand::ML_ANGLE_WITH_SPEED));
            message.payload[1] = std::byte {0};
            message.payload[2] = static_cast<std::byte>(max_speed & 0xFFU);
            message.payload[3] =
                static_cast<std::byte>((max_speed >> 8) & 0xFFU);
            message.payload[4] = static_cast<std::byte>(angle & 0xFFU);
            message.payload[5] = static_cast<std::byte>((angle >> 8) & 0xFFU);
            message.payload[6] = static_cast<std::byte>((angle >> 16) & 0xFFU);
            message.payload[7] = static_cast<std::byte>((angle >> 24) & 0xFFU);
        }

        void format_single_loop_message(uint32_t id, uint8_t spin_direction,
                                        uint16_t max_speed, uint32_t angle,
                                        MW_CAN::CANFrame& message) {
            message.sid = id;
            message.eid = 0U;
            message.is_extended_id = false;
            message.dlc = 8U;

            message.payload[0] = static_cast<std::byte>(
                static_cast<uint8_t>(MotorCommand::SL_ANGLE_WITH_SPEED));
            message.payload[1] = static_cast<std::byte>(spin_direction);
            message.payload[2] = static_cast<std::byte>(max_speed & 0xFFU);
            message.payload[3] =
                static_cast<std::byte>((max_speed >> 8) & 0xFFU);
            message.payload[4] = static_cast<std::byte>(angle & 0xFFU);
            message.payload[5] = static_cast<std::byte>((angle >> 8) & 0xFFU);
            message.payload[6] = static_cast<std::byte>((angle >> 16) & 0xFFU);
            message.payload[7] = static_cast<std::byte>((angle >> 24) & 0xFFU);
        }

        bool parse_single_loop_feedback(std::array<uint8_t, 4>& feedback,
                                        const MW_CAN::CANFrame& message) {
            ASSERT(message.dlc == 8,
                   "Invalid CAN message length for LK motor feedback");

            const uint8_t command =
                std::to_integer<uint8_t>(message.payload[0]);
            if (command != static_cast<uint8_t>(MotorCommand::READ_SL_FB)) {
                return false;
            }

            feedback[0] = std::to_integer<uint8_t>(message.payload[4]);
            feedback[1] = std::to_integer<uint8_t>(message.payload[5]);
            feedback[2] = std::to_integer<uint8_t>(message.payload[6]);
            feedback[3] = std::to_integer<uint8_t>(message.payload[7]);
            return true;
        }

        bool parse_raw_feedback(MotorFeedback& feedback,
                                const MW_CAN::CANFrame& message) {
            ASSERT(message.dlc == 8,
                   "Invalid CAN message length for LK motor feedback");

            const uint8_t command =
                std::to_integer<uint8_t>(message.payload[0]);
            if (command != static_cast<uint8_t>(MotorCommand::READ_FB_DATA)) {
                return false;
            }

            const auto byte_at = [&message](size_t index) -> uint16_t {
                return std::to_integer<uint8_t>(message.payload[index]);
            };

            feedback.angle =
                static_cast<int16_t>((byte_at(7) << 8) | byte_at(6));
            feedback.rpm = static_cast<int16_t>((byte_at(5) << 8) | byte_at(4));
            feedback.current =
                static_cast<int16_t>((byte_at(3) << 8) | byte_at(2));
            feedback.temperature = static_cast<int16_t>(byte_at(1));
            return true;
        }

        bool parse_encoder_feedback(uint16_t& encoder,
                                    const MW_CAN::CANFrame& message) {
            ASSERT(message.dlc == 8,
                   "Invalid CAN message length for LK motor feedback");

            const uint8_t command =
                std::to_integer<uint8_t>(message.payload[0]);
            if (command !=
                static_cast<uint8_t>(MotorCommand::READ_ENCODER_FB)) {
                return false;
            }

            const uint16_t msb = std::to_integer<uint8_t>(message.payload[3]);
            const uint16_t lsb = std::to_integer<uint8_t>(message.payload[2]);
            encoder = static_cast<uint16_t>((msb << 8) | lsb);
            return true;
        }

        bool parse_torque_feedback(TorqueFeedback& feedback,
                                   const MW_CAN::CANFrame& message) {
            ASSERT(message.dlc == 8,
                   "Invalid CAN message length for LK motor feedback");

            const uint8_t command =
                std::to_integer<uint8_t>(message.payload[0]);
            if (command !=
                static_cast<uint8_t>(MotorCommand::SL_ANGLE_WITH_SPEED)) {
                return false;
            }

            const auto byte_at = [&message](size_t index) -> uint16_t {
                return std::to_integer<uint8_t>(message.payload[index]);
            };

            feedback.temperature = static_cast<int8_t>(byte_at(1));
            feedback.torque_current =
                static_cast<int16_t>((byte_at(3) << 8) | byte_at(2));
            feedback.speed =
                static_cast<int16_t>((byte_at(5) << 8) | byte_at(4));
            feedback.ecd_position =
                static_cast<uint16_t>((byte_at(7) << 8) | byte_at(6));
            return true;
        }
    }  // namespace can
}  // namespace lk_motor

#endif