#ifndef __TOPICS__HPP
#define __TOPICS__HPP

#include <cstring>
#include "message_center.hpp"
#include "subsystems_types.hpp"
#include "uarm_lib.hpp"

namespace mc2 {
    struct ShootCommand
        : InterboardMessage<1, ShootCommand, MessageNode::Gimbal> {
        uint32_t command_bits;
        uint32_t extra_bits;

        static void serialize(const ShootCommand& msg, uint8_t* dst) {
            ASSERT((msg.extra_bits & 0xffff0000) == 0,
                   "Incoming extra_bits must not have 16 MSB set.");
            uint32_t encoded_command_bits = msg.command_bits;
            uint16_t encoded_extra_bits = msg.extra_bits & 0xffff;
            memcpy(dst, &encoded_command_bits, sizeof(uint32_t));
            memcpy(dst + sizeof(uint32_t), &encoded_extra_bits,
                   sizeof(uint16_t));
        }

        static void deserialize(ShootCommand& msg, const uint8_t* src) {
            uint32_t encoded_command_bits;
            uint16_t encoded_extra_bits;
            memcpy(&encoded_command_bits, src, sizeof(uint32_t));
            memcpy(&encoded_extra_bits, src + sizeof(uint32_t),
                   sizeof(uint16_t));
            msg.command_bits = encoded_command_bits;
            msg.extra_bits = encoded_extra_bits & 0xffff;
            ASSERT((msg.extra_bits & 0xffff0000) == 0,
                   "Incoming extra_bits must not have 16 MSB set.");
        }
    };

    struct GimbalCommand
        : InterboardMessage<1, GimbalCommand, MessageNode::Gimbal> {
        // yaw and pitch fields are in radians and requested changes in yaw/pitch (deltas).
        float yaw;
        float pitch;
        uint32_t command_bits;  // TODO: Implement and remove AUTO_AIM topic.

        static void serialize(const GimbalCommand& msg, uint8_t* dst) {
            ASSERT(-2 * PI < msg.yaw && msg.yaw < 2 * PI,
                   "Outgoing yaw out of acceptable range (-2PI, 2PI).");
            ASSERT(-2 * PI < msg.pitch && msg.pitch < 2 * PI,
                   "Outgoing pitch out of acceptable range (-2PI, 2PI).");
            ASSERT((msg.command_bits & 0xffff0000) == 0,
                   "Outgoing command_bits must not have 8 MSB set.");
            int16_t encoded_yaw = msg.yaw * 5000;
            int16_t encoded_pitch = msg.pitch * 5000;
            uint16_t encoded_command_bits = msg.command_bits & 0xffff;
            memcpy(dst, &encoded_yaw, sizeof(int16_t));
            memcpy(dst + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
            memcpy(dst + sizeof(int16_t) + sizeof(int16_t),
                   &encoded_command_bits, sizeof(uint16_t));
        }

        static void deserialize(GimbalCommand& msg, const uint8_t* src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            uint16_t encoded_command_bits;
            memcpy(&encoded_yaw, src, sizeof(int16_t));
            memcpy(&encoded_pitch, src + sizeof(int16_t), sizeof(int16_t));
            memcpy(&encoded_command_bits,
                   src + sizeof(int16_t) + sizeof(int16_t), sizeof(uint16_t));
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

    struct GimbalRelativeAngles
        : InterboardMessage<1, GimbalRelativeAngles, MessageNode::Chassis> {
        // yaw and pitch are relative to chassis front and in radians.

        // TODO: Verify sign conventions.
        // Sign convention follows CCW-positive for yaw, and up-positive for pitch.
        float yaw;
        float pitch;

        static void serialize(const GimbalRelativeAngles& msg, uint8_t* dst) {
            ASSERT(-PI < msg.yaw && msg.yaw < PI,
                   "Outgoing yaw out of acceptable range (-PI, PI).");
            ASSERT(-PI < msg.pitch && msg.pitch < PI,
                   "Outgoing pitch out of acceptable range (-PI, PI).");
            int16_t encoded_yaw = msg.yaw * 10000;
            int16_t encoded_pitch = msg.pitch * 10000;
            memcpy(dst, &encoded_yaw, sizeof(int16_t));
            memcpy(dst + sizeof(int16_t), &encoded_pitch, sizeof(int16_t));
        }

        static void deserialize(GimbalRelativeAngles& msg, const uint8_t* src) {
            int16_t encoded_yaw;
            int16_t encoded_pitch;
            memcpy(&encoded_yaw, src, sizeof(int16_t));
            memcpy(&encoded_pitch, src + sizeof(int16_t), sizeof(int16_t));
            msg.yaw = static_cast<float>(encoded_yaw) / 10000;
            msg.pitch = static_cast<float>(encoded_pitch) / 10000;
            ASSERT(-PI < msg.yaw && msg.yaw < PI,
                   "Incoming yaw out of acceptable range (-PI, PI).");
            ASSERT(-PI < msg.pitch && msg.pitch < PI,
                   "Incoming pitch out of acceptable range (-PI, PI).");
        }
    };

    struct RefereeInfo
        : InterboardMessage<1, RefereeInfo, MessageNode::Gimbal> {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t shoot_barrel_cooling_rate;
        uint16_t shoot_barrel_heat_limit;
        uint16_t chassis_power_limit;

        static void serialize(const RefereeInfo& msg, uint8_t* dst) {
            (void) msg;
            (void) dst;
        }

        static void deserialize(RefereeInfo& msg, const uint8_t* src) {
            (void) msg;
            (void) src;
        }
    };

    struct MotorSet : Topic<5> {
        int32_t motor_can_volts[8];
        Motor_CAN_ID_t can_ids[8];
    };

    struct MotorRead : Topic<1> {
        uint8_t feedback[8][8];
        Motor_CAN_ID_t can_ids[8];
    };

    struct ChassisCommand : Topic<1> {
        float v_perp;
        float v_parallel;
        float wz;
        uint16_t command_bits;
    };

    struct CommOut : Topic<5> {
        uint32_t topic_name;
        std::array<uint8_t, 8> bytes;
    };

    struct CommIn : Topic<5> {
        uint32_t topic_name;
        std::array<uint8_t, 8> bytes;
    };

    struct ImuReadings : Topic<1> {
        float yaw;
        float pitch;
    };

    struct RefereeIn : Topic<1> {
        std::array<uint8_t, 41> ref_bytes;
    };

    struct RefereeOut : Topic<1> {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t shoot_barrel_cooling_rate;
        uint16_t shoot_barrel_heat_limit;
        uint16_t chassis_power_limit;
    };

    struct RCRaw : Topic<1> {
        std::array<uint8_t, 18> rc_bytes;
    };

    struct AutoAim : Topic<1> {
        uint8_t target_num;
        bool should_shoot;
        float delta_yaw;
        float delta_pitch;
    };

    struct UCPackIn : Topic<1> {
        std::array<uint8_t, 64> bytes;
    };

    struct UCPackOut : Topic<10> {
        std::array<uint8_t, 192> bytes;
    };

    // TODO: Add create registry template function to impose restrictions on valid types
    // for use as topics. Check that all types eventually derive from Topic<> and that
    // topics that derive from InterboardMessage<> implement serializer() and deserialize()
    // static methods.
    using RobotTopics =
        std::tuple<MotorSet, MotorRead, ChassisCommand, GimbalCommand,
                   ShootCommand, RefereeInfo, CommOut, CommIn, ImuReadings,
                   GimbalRelativeAngles, RefereeIn, RefereeOut, RCRaw, AutoAim,
                   UCPackIn, UCPackOut>;

    using RobotMC = MC2<RobotTopics>;
}  // namespace mc2

#endif
