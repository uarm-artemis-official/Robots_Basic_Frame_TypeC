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

namespace dji_motors {
    enum class MotorType { GM6020, GM3510, M2006 };
    struct MotorFeedback {
        int16_t encoder_angle;
        int16_t rpm;
        int16_t current;
        int16_t temperature;
    };

    void format_voltage_message(MotorType motor_type, int32_t v1, int32_t v2,
                                int32_t v3, int32_t v4,
                                MW_CAN::CANFrame& message);

    bool parse_feedback_message(MotorFeedback& feedback,
                                const MW_CAN::CANFrame& message);
}  // namespace dji_motors

#endif