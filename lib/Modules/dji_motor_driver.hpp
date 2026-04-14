// DJI Motor Control Headers
// Used for controlling DJI motors, such as GM6020, GM3510, etc.
// Motors are controlled via CAN bus in boardcast mode (i.e. multiple
// motors can be controlled with a single message).
// A maximum of 8 motors (sometimes fewer, depending on motor type)
// can be controlled on the same CAN bus at a time due to ID limitations.

#ifndef DJI_MOTORS_HPP
#define DJI_MOTORS_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace dji_motor {
    enum class MotorType { GM6020, GM3510, M2006 };
    struct MotorFeedback {
        int16_t encoder_angle;
        int16_t rpm;
        int16_t current;
        int16_t temperature;
    };

    void format_voltage_message(MotorType motor_type, int32_t v1, int32_t v2,
                                int32_t v3, int32_t v4,
                                MW_CAN::CANFrame& message) {
        message.sid = (motor_type == MotorType::GM3510) ? 0x200U : 0x1FFU;
        message.eid = 0U;
        message.is_extended_id = false;
        message.dlc = 0x08U;

        message.payload[0] = static_cast<std::byte>((v1 >> 8) & 0xFF);
        message.payload[1] = static_cast<std::byte>(v1 & 0xFF);
        message.payload[2] = static_cast<std::byte>((v2 >> 8) & 0xFF);
        message.payload[3] = static_cast<std::byte>(v2 & 0xFF);
        message.payload[4] = static_cast<std::byte>((v3 >> 8) & 0xFF);
        message.payload[5] = static_cast<std::byte>(v3 & 0xFF);
        message.payload[6] = static_cast<std::byte>((v4 >> 8) & 0xFF);
        message.payload[7] = static_cast<std::byte>(v4 & 0xFF);
    }

    bool parse_feedback_message(MotorFeedback& feedback,
                                const MW_CAN::CANFrame& message) {
        ASSERT(message.dlc == 8,
               "Invalid CAN message length for motor feedback");

        const auto byte_at = [&message](size_t index) -> uint16_t {
            return std::to_integer<uint8_t>(message.payload[index]);
        };

        feedback.encoder_angle =
            static_cast<int16_t>((byte_at(0) << 8) | byte_at(1));
        feedback.rpm = static_cast<int16_t>((byte_at(2) << 8) | byte_at(3));
        feedback.current = static_cast<int16_t>((byte_at(4) << 8) | byte_at(5));
        feedback.temperature = static_cast<int16_t>(byte_at(6));
        return true;
    }
}  // namespace dji_motor

#endif