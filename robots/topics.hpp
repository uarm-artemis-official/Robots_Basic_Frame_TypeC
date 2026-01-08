#ifndef __TOPICS__HPP
#define __TOPICS__HPP

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include "message_center.hpp"
#include "subsystems_types.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

namespace mc2 {
    struct ShootCommand {
        static constexpr size_t queue_size = 1;
        static constexpr size_t serialized_size = 6;
        static constexpr MessageNode destination = MessageNode::Gimbal;

        uint32_t command_bits;
        uint32_t extra_bits;

        static bool serialize(const ShootCommand& msg,
                              std::span<std::byte, serialized_size> dst) {
            ASSERT((msg.extra_bits & 0xffff0000) == 0,
                   "Incoming extra_bits must not have 16 MSB set.");
            uint32_t encoded_command_bits = msg.command_bits;
            uint16_t encoded_extra_bits = msg.extra_bits & 0xffff;
            dst[0] =
                std::byte {static_cast<uint8_t>(encoded_command_bits & 0xFF)};
            dst[1] = std::byte {
                static_cast<uint8_t>((encoded_command_bits >> 8) & 0xFF)};
            dst[2] = std::byte {
                static_cast<uint8_t>((encoded_command_bits >> 16) & 0xFF)};
            dst[3] = std::byte {
                static_cast<uint8_t>((encoded_command_bits >> 24) & 0xFF)};

            dst[4] =
                std::byte {static_cast<uint8_t>(encoded_extra_bits & 0xFF)};
            dst[5] = std::byte {
                static_cast<uint8_t>((encoded_extra_bits >> 8) & 0xFF)};
            return true;
        }

        static bool deserialize(
            ShootCommand& msg,
            std::span<const std::byte, serialized_size> src) {
            uint32_t encoded_command_bits;
            uint16_t encoded_extra_bits;
            encoded_command_bits =
                static_cast<uint32_t>(std::to_integer<uint8_t>(src[0])) |
                (static_cast<uint32_t>(std::to_integer<uint8_t>(src[1])) << 8) |
                (static_cast<uint32_t>(std::to_integer<uint8_t>(src[2]))
                 << 16) |
                (static_cast<uint32_t>(std::to_integer<uint8_t>(src[3])) << 24);
            encoded_extra_bits =
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[4])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[5])) << 8);
            msg.command_bits = encoded_command_bits;
            msg.extra_bits = encoded_extra_bits & 0xffff;
            return true;
        }
    };

    struct GimbalCommand {
        static constexpr size_t queue_size = 1;
        static constexpr size_t serialized_size = 6;
        static constexpr MessageNode destination = MessageNode::Gimbal;

        // yaw and pitch fields are in radians and requested changes in yaw/pitch (deltas).
        float yaw;
        float pitch;
        uint32_t command_bits;  // TODO: Implement and remove AUTO_AIM topic.

        static bool serialize(const GimbalCommand& msg,
                              std::span<std::byte, serialized_size> dst) {
            ASSERT(-2 * PI < msg.yaw && msg.yaw < 2 * PI,
                   "Outgoing yaw out of acceptable range (-2PI, 2PI).");
            ASSERT(-2 * PI < msg.pitch && msg.pitch < 2 * PI,
                   "Outgoing pitch out of acceptable range (-2PI, 2PI).");
            ASSERT((msg.command_bits & 0xffff0000) == 0,
                   "Outgoing command_bits must not have 8 MSB set.");
            int16_t encoded_yaw = value_limit(msg.yaw, -PI * 2, PI * 2) * 5000;
            int16_t encoded_pitch =
                value_limit(msg.pitch, -PI * 2, PI * 2) * 5000;
            uint16_t encoded_command_bits = msg.command_bits & 0xffff;
            dst[0] = std::byte {static_cast<uint8_t>(encoded_yaw & 0xFF)};
            dst[1] =
                std::byte {static_cast<uint8_t>((encoded_yaw >> 8) & 0xFF)};
            dst[2] = std::byte {static_cast<uint8_t>(encoded_pitch & 0xFF)};
            dst[3] =
                std::byte {static_cast<uint8_t>((encoded_pitch >> 8) & 0xFF)};
            dst[4] =
                std::byte {static_cast<uint8_t>(encoded_command_bits & 0xFF)};
            dst[5] = std::byte {
                static_cast<uint8_t>((encoded_command_bits >> 8) & 0xFF)};
            return true;
        }

        static bool deserialize(
            GimbalCommand& msg,
            std::span<const std::byte, serialized_size> src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            uint16_t encoded_command_bits;

            encoded_yaw = static_cast<int16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[0])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[1])) << 8));
            encoded_pitch = static_cast<int16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[2])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[3])) << 8));
            encoded_command_bits = static_cast<uint16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[4])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[5])) << 8));
            msg.yaw = static_cast<float>(encoded_yaw) / 5000;
            msg.pitch = static_cast<float>(encoded_pitch) / 5000;
            msg.command_bits = static_cast<uint32_t>(encoded_command_bits);
            ASSERT(-2 * PI < msg.yaw && msg.yaw < 2 * PI,
                   "Incoming yaw out of acceptable range (-2PI, 2PI).");
            ASSERT(-2 * PI < msg.pitch && msg.pitch < 2 * PI,
                   "Incoming pitch out of acceptable range (-2PI, 2PI).");
            ASSERT((msg.command_bits & 0xffff0000) == 0,
                   "Incoming command_bits must not have 8 MSB set.");
            return true;
        }
    };

    struct GimbalRelativeAngles {
        static constexpr size_t queue_size = 1;
        static constexpr size_t serialized_size = 4;
        static constexpr MessageNode destination = MessageNode::Chassis;

        // yaw and pitch are relative to chassis front and in radians.

        // TODO: Verify sign conventions.
        // Sign convention follows CCW-positive for yaw, and up-positive for pitch.
        float yaw;
        float pitch;

        static bool serialize(const GimbalRelativeAngles& msg,
                              std::span<std::byte, serialized_size> dst) {
            constexpr float TOLERANCE = 0.0001f;
            constexpr float endpoint_magnitude = PI + TOLERANCE;
            ASSERT(
                -endpoint_magnitude < msg.yaw && msg.yaw < endpoint_magnitude,
                "Outgoing yaw out of acceptable range (-PI, PI).");
            ASSERT(-endpoint_magnitude < msg.pitch &&
                       msg.pitch < endpoint_magnitude,
                   "Outgoing pitch out of acceptable range (-PI, PI).");
            int16_t encoded_yaw = value_limit(msg.yaw, -PI, PI) * 10000;
            int16_t encoded_pitch = value_limit(msg.pitch, -PI, PI) * 10000;
            dst[0] = std::byte {static_cast<uint8_t>(encoded_yaw & 0xFF)};
            dst[1] =
                std::byte {static_cast<uint8_t>((encoded_yaw >> 8) & 0xFF)};
            dst[2] = std::byte {static_cast<uint8_t>(encoded_pitch & 0xFF)};
            dst[3] =
                std::byte {static_cast<uint8_t>((encoded_pitch >> 8) & 0xFF)};
            return true;
        }

        static bool deserialize(
            GimbalRelativeAngles& msg,
            std::span<const std::byte, serialized_size> src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            encoded_yaw = static_cast<int16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[0])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[1])) << 8));
            encoded_pitch = static_cast<int16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[2])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[3])) << 8));
            msg.yaw =
                value_limit(static_cast<float>(encoded_yaw) / 10000, -PI, PI);
            msg.pitch =
                value_limit(static_cast<float>(encoded_pitch) / 10000, -PI, PI);

            constexpr float TOLERANCE = 0.0001f;
            constexpr float endpoint_magnitude = PI + TOLERANCE;
            ASSERT(
                -endpoint_magnitude < msg.yaw && msg.yaw < endpoint_magnitude,
                "Incoming yaw out of acceptable range (-PI, PI).");
            ASSERT(-endpoint_magnitude < msg.pitch &&
                       msg.pitch < endpoint_magnitude,
                   "Incoming pitch out of acceptable range (-PI, PI).");
            return true;
        }
    };

    struct RefereeInfo {
        static constexpr size_t queue_size = 1;
        static constexpr size_t serialized_size = 8;
        static constexpr MessageNode destination = MessageNode::Gimbal;

        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t shoot_barrel_cooling_rate;
        uint16_t shoot_barrel_heat_limit;
        uint16_t chassis_power_limit;

        static bool serialize(const RefereeInfo& msg,
                              std::span<std::byte, serialized_size> dst) {
            (void) msg;
            (void) dst;
            ASSERT(false, "");
            return false;
        }

        static bool deserialize(
            RefereeInfo& msg, std::span<const std::byte, serialized_size> src) {
            (void) msg;
            (void) src;
            ASSERT(false, "");
            return false;
        }
    };

    struct MotorSet {
        static constexpr size_t queue_size = 5;

        int32_t motor_can_volts[8];
        Motor_CAN_ID_t can_ids[8];
    };

    struct MotorRead {
        static constexpr size_t queue_size = 1;

        uint8_t feedback[8][8];
        Motor_CAN_ID_t can_ids[8];
    };

    struct ChassisCommand {
        static constexpr size_t queue_size = 1;

        float v_perp;
        float v_parallel;
        float wz;
        uint16_t command_bits;
    };

    struct CommOut {
        static constexpr size_t queue_size = 5;

        uint32_t topic_name;
        std::array<uint8_t, 8> bytes;
    };

    struct CommIn {
        static constexpr size_t queue_size = 5;

        uint32_t topic_name;
        std::array<uint8_t, 8> bytes;
    };

    struct ImuReadings {
        static constexpr size_t queue_size = 1;

        float yaw;
        float pitch;
    };

    struct RefereeIn {
        static constexpr size_t queue_size = 1;

        std::array<uint8_t, 41> ref_bytes;
    };

    struct RefereeOut {
        static constexpr size_t queue_size = 1;

        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t shoot_barrel_cooling_rate;
        uint16_t shoot_barrel_heat_limit;
        uint16_t chassis_power_limit;
    };

    struct RCRaw {
        static constexpr size_t queue_size = 1;

        std::array<uint8_t, 18> rc_bytes;
    };

    struct AutoAim {
        static constexpr size_t queue_size = 1;

        uint8_t target_num;
        bool should_shoot;
        float delta_yaw;
        float delta_pitch;
    };

    struct UCPackIn {
        static constexpr size_t queue_size = 1;

        std::array<uint8_t, 64> bytes;
    };

    struct UCPackOut {
        static constexpr size_t queue_size = 10;

        std::array<uint8_t, 192> bytes;
    };

    // TODO: Add create registry template function to impose restrictions on valid types
    // for use as topics. Check that all types eventually derive from Topic<> and that
    // topics that derive from InterboardMessage<> implement serialize() and deserialize()
    // static methods.
    using RobotTopics = create_topic_registry_t<
        MotorSet, MotorRead, ChassisCommand, GimbalCommand, ShootCommand,
        RefereeInfo, CommOut, CommIn, ImuReadings, GimbalRelativeAngles,
        RefereeIn, RefereeOut, RCRaw, AutoAim, UCPackIn, UCPackOut>;

    using RobotMC = MC2<RobotTopics>;
}  // namespace mc2

#endif
