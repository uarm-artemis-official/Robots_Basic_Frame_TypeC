#ifndef __SENTRY_CONFIG_HPP
#define __SENTRY_CONFIG_HPP

#include "apps_types.hpp"

static const apps::gimbal::GimbalConfig gimbal_config = {
    .yaw_position_pid_config =
        pid::PID2Config {
            .k_p = 105,
            .k_i = 0,
            .k_d = 0.05,
            .beta = 1,
            .yeta = 0,
            .max_out = 800,
            .min_out = -800,
        },
    .yaw_speed_pid_config =
        pid::PID2Config {
            .k_p = 240,
            .k_i = 10,
            .k_d = 0,
            .beta = 1,
            .yeta = 1,
            .max_out = 20000,
            .min_out = -20000,
        },
    .pitch_position_pid_config =
        pid::PID2Config {
            .k_p = 250,
            .k_i = 0,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 1000,
            .min_out = -1000,
        },
    .pitch_speed_pid_config =
        pid::PID2Config {
            .k_p = 250,
            .k_i = 50,
            .k_d = 0,
            .beta = 1,
            .yeta = 0,
            .max_out = 20000,
            .min_out = -20000,
        },
    .yaw_ecd_center = 4430,
    .pitch_ecd_center = 4050,
    .upwards_pitch_orientation = 1,
    .max_pitch_angle = 0.1,
    .min_pitch_angle = -0.2,
    .imu_orientation =
        {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        },
    .imu_yaw_lpf_gain = 1.0f,
    .imu_pitch_lpf_gain = 0.90f,
    .exit_calibration_yaw_angle_delta = uarm::strong_types::Degree(2.0f),
};

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
    .loader_gear_ratio = 36.0f,
    .jam_stall_duration_threshold = uarm::strong_types::Second(0.5),
    .jam_loader_rpm_threshold = uarm::strong_types::RotationsPerMinute(5),
    .jam_loader_current_relative_diff_threshold = 0.05f,
    .jam_no_stall_duration_threshold = uarm::strong_types::Second(0.5),
    .enable_loader_position_control = false,
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
            .max_out = 2000,
            .min_out = -2000,
        },
    .max_translation_speed = uarm::strong_types::MetersPerSecond(8),
    .max_rotation_speed = uarm::strong_types::RadiansPerSecond(5),
    .gyro_speed = uarm::strong_types::RadiansPerSecond(5),
    .max_wheel_ramp_accel = uarm::strong_types::RadiansPerSecondSecond(400),
};

#endif
