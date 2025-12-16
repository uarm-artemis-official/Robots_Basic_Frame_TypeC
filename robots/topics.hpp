#ifndef __TOPICS__HPP
#define __TOPICS__HPP

#include <cstdint>
#include <cstring>
#include <span>
#include "message_center.hpp"
#include "subsystems_types.hpp"
#include "uarm_lib.hpp"

namespace mc2 {
    struct ShootCommand {
        static constexpr size_t queue_size = 1;
        static constexpr size_t serialized_size = 6;
        static constexpr MessageNode destination = MessageNode::Gimbal;

        uint32_t command_bits;
        uint32_t extra_bits;

        static void serialize(const ShootCommand& msg,
                              std::span<uint8_t, serialized_size> dst) {
            ASSERT((msg.extra_bits & 0xffff0000) == 0,
                   "Incoming extra_bits must not have 16 MSB set.");
            uint32_t encoded_command_bits = msg.command_bits;
            uint16_t encoded_extra_bits = msg.extra_bits & 0xffff;

            dst[0] = encoded_command_bits & 0xFF;
            dst[1] = (encoded_command_bits >> 8) & 0xFF;
            dst[2] = (encoded_command_bits >> 16) & 0xFF;
            dst[3] = (encoded_command_bits >> 24) & 0xFF;

            dst[4] = encoded_extra_bits & 0xFF;
            dst[5] = (encoded_extra_bits >> 8) & 0xFF;
        }

        static void deserialize(ShootCommand& msg,
                                std::span<const uint8_t, serialized_size> src) {
            uint32_t encoded_command_bits;
            uint16_t encoded_extra_bits;
            encoded_command_bits = static_cast<uint32_t>(src[0]) |
                                   (static_cast<uint32_t>(src[1]) << 8) |
                                   (static_cast<uint32_t>(src[2]) << 16) |
                                   (static_cast<uint32_t>(src[3]) << 24);
            encoded_extra_bits = static_cast<uint16_t>(src[4]) |
                                 (static_cast<uint16_t>(src[5]) << 8);
            msg.command_bits = encoded_command_bits;
            msg.extra_bits = encoded_extra_bits & 0xffff;
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

        static void serialize(const GimbalCommand& msg,
                              std::span<uint8_t, serialized_size> dst) {
            ASSERT(-2 * PI < msg.yaw && msg.yaw < 2 * PI,
                   "Outgoing yaw out of acceptable range (-2PI, 2PI).");
            ASSERT(-2 * PI < msg.pitch && msg.pitch < 2 * PI,
                   "Outgoing pitch out of acceptable range (-2PI, 2PI).");
            ASSERT((msg.command_bits & 0xffff0000) == 0,
                   "Outgoing command_bits must not have 8 MSB set.");
            int16_t encoded_yaw = msg.yaw * 5000;
            int16_t encoded_pitch = msg.pitch * 5000;
            uint16_t encoded_command_bits = msg.command_bits & 0xffff;

            dst[0] = encoded_yaw & 0xFF;
            dst[1] = (encoded_yaw >> 8) & 0xFF;
            dst[2] = encoded_pitch & 0xFF;
            dst[3] = (encoded_pitch >> 8) & 0xFF;
            dst[4] = encoded_command_bits & 0xFF;
            dst[5] = (encoded_command_bits >> 8) & 0xFF;
        }

        static void deserialize(GimbalCommand& msg,
                                std::span<const uint8_t, serialized_size> src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            uint16_t encoded_command_bits;

            encoded_yaw = static_cast<int16_t>(src[0]) |
                          (static_cast<int16_t>(src[1]) << 8);
            encoded_pitch = static_cast<int16_t>(src[2]) |
                            (static_cast<int16_t>(src[3]) << 8);
            encoded_command_bits = static_cast<uint16_t>(src[4]) |
                                   (static_cast<uint16_t>(src[5]) << 8);
            msg.yaw = static_cast<float>(encoded_yaw) / 5000;
            msg.pitch = static_cast<float>(encoded_pitch) / 5000;
            msg.command_bits = static_cast<uint32_t>(encoded_command_bits);
            ASSERT(-2 * PI < msg.yaw && msg.yaw < 2 * PI,
                   "Incoming yaw out of acceptable range (-2PI, 2PI).");
            ASSERT(-2 * PI < msg.pitch && msg.pitch < 2 * PI,
                   "Incoming pitch out of acceptable range (-2PI, 2PI).");
            ASSERT((msg.command_bits & 0xffff0000) == 0,
                   "Incoming command_bits must not have 8 MSB set.");
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

        static void serialize(const GimbalRelativeAngles& msg,
                              std::span<uint8_t, serialized_size> dst) {
            ASSERT(-PI < msg.yaw && msg.yaw < PI,
                   "Outgoing yaw out of acceptable range (-PI, PI).");
            ASSERT(-PI < msg.pitch && msg.pitch < PI,
                   "Outgoing pitch out of acceptable range (-PI, PI).");
            int16_t encoded_yaw = msg.yaw * 10000;
            int16_t encoded_pitch = msg.pitch * 10000;
            dst[0] = encoded_yaw & 0xFF;
            dst[1] = (encoded_yaw >> 8) & 0xFF;
            dst[2] = encoded_pitch & 0xFF;
            dst[3] = (encoded_pitch >> 8) & 0xFF;
        }

        static void deserialize(GimbalRelativeAngles& msg,
                                std::span<const uint8_t, serialized_size> src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            encoded_yaw = static_cast<int16_t>(src[0]) |
                          (static_cast<int16_t>(src[1]) << 8);
            encoded_pitch = static_cast<int16_t>(src[2]) |
                            (static_cast<int16_t>(src[3]) << 8);
            msg.yaw = static_cast<float>(encoded_yaw) / 10000;
            msg.pitch = static_cast<float>(encoded_pitch) / 10000;
            ASSERT(-PI < msg.yaw && msg.yaw < PI,
                   "Incoming yaw out of acceptable range (-PI, PI).");
            ASSERT(-PI < msg.pitch && msg.pitch < PI,
                   "Incoming pitch out of acceptable range (-PI, PI).");
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

        static void serialize(const RefereeInfo& msg,
                              std::span<uint8_t, serialized_size> dst) {
            (void) msg;
            (void) dst;
        }

        static void deserialize(RefereeInfo& msg,
                                std::span<const uint8_t, serialized_size> src) {
            (void) msg;
            (void) src;
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
