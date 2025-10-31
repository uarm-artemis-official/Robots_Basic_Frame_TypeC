#ifndef __TOPICS__HPP
#define __TOPICS__HPP

#include "message_center.hpp"
#include "subsystems_types.hpp"

namespace mc2 {
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

    struct GimbalCommand
        : Topic<1>,
          InterboardMessage<GimbalCommand, MessageNode::Gimbal> {
        // yaw and pitch fields are in radians and requested changes in yaw/pitch (deltas).
        float yaw;
        float pitch;
        uint32_t command_bits;  // TODO: Implement and remove AUTO_AIM topic.

        void encode(std::array<uint8_t, 200>& bytes) override;
        void decode(std::array<uint8_t, 200>& bytes) override;
    };

    struct ShootCommand : Topic<1>,
                          InterboardMessage<ShootCommand, MessageNode::Gimbal> {
        uint32_t command_bits;
        uint32_t extra_bits;

        void encode(std::array<uint8_t, 200>& bytes) override;
        void decode(std::array<uint8_t, 200>& bytes) override;
    };

    struct RefereeInfo : Topic<1>,
                         InterboardMessage<RefereeInfo, MessageNode::Gimbal> {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t shoot_barrel_cooling_rate;
        uint16_t shoot_barrel_heat_limit;
        uint16_t chassis_power_limit;

        // TODO: Implement.
        void encode(std::array<uint8_t, 200>& bytes) override;
        void decode(std::array<uint8_t, 200>& bytes) override;
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

    struct GimbalRelativeAngles
        : Topic<1>,
          InterboardMessage<GimbalRelativeAngles, MessageNode::Chassis> {
        // yaw and pitch are relative to chassis front and in radians.

        // TODO: Verify sign conventions.
        // Sign convention follows CCW-positive for yaw, and up-positive for pitch.
        float yaw;
        float pitch;

        void encode(std::array<uint8_t, 200>& bytes) override;
        void decode(std::array<uint8_t, 200>& bytes) override;
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

    using RobotTopics =
        std::tuple<MotorSet, MotorRead, ChassisCommand, GimbalCommand,
                   ShootCommand, RefereeInfo, CommOut, CommIn, ImuReadings,
                   GimbalRelativeAngles, RefereeIn, RefereeOut, RCRaw, AutoAim,
                   UCPackIn, UCPackOut>;

    using RobotMC = MC2<RobotTopics>;
}  // namespace mc2

#endif
