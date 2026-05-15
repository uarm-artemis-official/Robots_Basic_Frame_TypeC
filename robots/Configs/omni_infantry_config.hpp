#ifndef __OMNI_INFANTRY_CONFIG_H
#define __OMNI_INFANTRY_CONFIG_H

#include <cstdint>

namespace gimbal_params {
    // Motor encoder zero-position.
    constexpr int16_t YAW_ECD_CENTER = 7815;
    constexpr int16_t PITCH_ECD_CENTER = 4800;
    constexpr float LOADER_GEAR_RATIO = 36;

    // Software pitch limits.
    constexpr float PITCH_MIN_ANGLE = -0.14;
    constexpr float PITCH_MAX_ANGLE = 0.10;
    static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);

    // Orientation
    // BMI088 rotation matrix.
    // TODO: Move to front-right-up (x-y-z) reference frame???
    constexpr float IMU_ORIENTATION[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    // 1 = CCW positive (i.e. rotating CCW pitches up)
    // -1 = CW positive (i.e. rotating CW pitches up)
    constexpr float PITCH_ORIENTATION = -1;

    // Gimbal yaw PID.
    constexpr float KP_GIMBAL_YAW_ANGLE = 150;
    constexpr float KI_GIMBAL_YAW_ANGLE = 0;  // Not allowed.
    constexpr float KD_GIMBAL_YAW_ANGLE = 0;
    constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
    constexpr float YETA_GIMBAL_YAW_ANGLE = 0;
    constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = -800;
    constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 800;
    static_assert(MAX_OUT_GIMBAL_YAW_ANGLE >= MIN_OUT_GIMBAL_YAW_ANGLE);

    constexpr float KP_GIMBAL_YAW_SPEED = 220;
    constexpr float KI_GIMBAL_YAW_SPEED = 30;
    constexpr float KD_GIMBAL_YAW_SPEED = 0;
    constexpr float BETA_GIMBAL_YAW_SPEED = 1;
    constexpr float YETA_GIMBAL_YAW_SPEED = 1;
    constexpr float MIN_OUT_GIMBAL_YAW_SPEED = -20000;
    constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 20000;
    // constexpr float MIN_OUT_GIMBAL_YAW_SPEED = 0;
    // constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 0;
    static_assert(MAX_OUT_GIMBAL_YAW_SPEED >= MIN_OUT_GIMBAL_YAW_SPEED);

    // Gimbal pitch PID.
    // constexpr float KP_GIMBAL_PITCH_ANGLE = 220;
    // constexpr float KI_GIMBAL_PITCH_ANGLE = 12;
    // constexpr float KD_GIMBAL_PITCH_ANGLE = 2;
    // constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
    // constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
    // constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
    // constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;
    // static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

    // constexpr float KP_GIMBAL_PITCH_SPEED = 80;
    // constexpr float KI_GIMBAL_PITCH_SPEED = 10;
    // constexpr float KD_GIMBAL_PITCH_SPEED = 0.05;
    // constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
    // constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
    // constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
    // constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;

    // Gimbal pitch PID.
    constexpr float KP_GIMBAL_PITCH_ANGLE = 220;  //100 220
    constexpr float KI_GIMBAL_PITCH_ANGLE = 0;    //0 12
    constexpr float KD_GIMBAL_PITCH_ANGLE = 10;   //0.05 2
    constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
    constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
    constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
    constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;
    static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

    constexpr float KP_GIMBAL_PITCH_SPEED = 100;  //120 80
    constexpr float KI_GIMBAL_PITCH_SPEED = 40;   //8 10
    constexpr float KD_GIMBAL_PITCH_SPEED = 0;    //0.01 0.05
    constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
    constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
    constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -30000;
    constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 30000;
    static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
    static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
}  // namespace gimbal_params

#include "apps_types.hpp"

static const apps::shoot::ShootConfig shoot_config = {
    .flywheel_config = apps::shoot::FlywheelConfiguration::DUAL,
    .flywheel_speed_pid_config =
        pid::PID2Config {
            .k_p = 27,
            .k_i = 5,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 5000,
            .min_out = -5000,
        },
    .loader_speed_pid_config =
        pid::PID2Config {
            .k_p = 30,
            .k_i = 0,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 10000,
            .min_out = -10000,
        },
    .loader_position_pid_config =
        pid::PID2Config {
            .k_p = 0,
            .k_i = 0,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 10000,
            .min_out = -10000,
        },
    .active_loader_speed = uarm::strong_types::RotationsPerMinute(75),
    .active_flywheel_speed = uarm::strong_types::RotationsPerMinute(7000),
    .max_flywheel_accel = uarm::strong_types::RotationsPerMinuteSecond(70000),
};

static const apps::chassis::ChassisConfig chassis_config = {
    .spin_pid_config =
        pid::PID2Config {
            .k_p = 1,
            .k_i = 0,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 2,
            .min_out = -2,
        },
    .wheel_pid_config =
        pid::PID2Config {
            .k_p = 5,
            .k_i = 0,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 5000,
            .min_out = -5000,
        },
    .max_translation_speed = uarm::strong_types::MetersPerSecond(8),
    .max_rotation_speed = uarm::strong_types::RadiansPerSecond(5),
    .gyro_speed = uarm::strong_types::RadiansPerSecond(5),
    .max_wheel_ramp_accel = uarm::strong_types::RadiansPerSecondSecond(400),
};

#endif