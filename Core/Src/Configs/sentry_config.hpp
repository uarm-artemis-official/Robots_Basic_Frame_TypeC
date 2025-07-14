#ifndef __SENTRY_CONFIG_HPP
#define __SENTRY_CONFIG_HPP

#include <cstdint>

namespace gimbal_params {
    // Motor encoder zero-position.
    constexpr int16_t YAW_ECD_CENTER = 4430;
    constexpr int16_t PITCH_ECD_CENTER = 4050;
    constexpr float LOADER_GEAR_RATIO = 36;

    // Software pitch limits.
    constexpr float PITCH_MIN_ANGLE = -0.2;
    constexpr float PITCH_MAX_ANGLE = 0.1;
    static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);

    // BMI088 rotation matrix.
    // TODO: Move to front-right-up (x-y-z) reference frame???
    constexpr float IMU_ORIENTATION[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };

    // Gimbal yaw PID.
    constexpr float KP_GIMBAL_YAW_ANGLE = 105;
    constexpr float KI_GIMBAL_YAW_ANGLE = 0;  // Not allowed.
    constexpr float KD_GIMBAL_YAW_ANGLE = 0.05;
    constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
    constexpr float YETA_GIMBAL_YAW_ANGLE = 0;
    constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = -800;
    constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 800;
    static_assert(MAX_OUT_GIMBAL_YAW_ANGLE >= MIN_OUT_GIMBAL_YAW_ANGLE);

    constexpr float KP_GIMBAL_YAW_SPEED = 240;
    constexpr float KI_GIMBAL_YAW_SPEED = 10;
    constexpr float KD_GIMBAL_YAW_SPEED = 0;
    constexpr float BETA_GIMBAL_YAW_SPEED = 1;
    constexpr float YETA_GIMBAL_YAW_SPEED = 1;
    constexpr float MIN_OUT_GIMBAL_YAW_SPEED = -20000;
    constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 20000;
    static_assert(MAX_OUT_GIMBAL_YAW_SPEED >= MIN_OUT_GIMBAL_YAW_SPEED);

    // Gimbal pitch PID.
    constexpr float KP_GIMBAL_PITCH_ANGLE = 80;
    constexpr float KI_GIMBAL_PITCH_ANGLE = 10;
    constexpr float KD_GIMBAL_PITCH_ANGLE = 0.05;
    constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
    constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
    constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
    constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;
    static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

    constexpr float KP_GIMBAL_PITCH_SPEED = 200;
    constexpr float KI_GIMBAL_PITCH_SPEED = 10;
    constexpr float KD_GIMBAL_PITCH_SPEED = 0;
    constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
    constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
    constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
    constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;
    static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);

}  // namespace gimbal_params

namespace shoot_params {
    // Shoot targets.
    constexpr float LOADER_ACTIVE_RPM = 75;
    constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 7000;
    constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
    static_assert(MAX_FLYWHEEL_ACCEL >= 0);

    // Loader PID.
    constexpr float KP_LOADER_SPEED = 12;
    constexpr float KI_LOADER_SPEED = 0;
    constexpr float KD_LOADER_SPEED = 0;
    constexpr float BETA_LOADER_SPEED = 1;
    constexpr float YETA_LOADER_SPEED = 0;
    constexpr float MIN_OUT_LOADER_SPEED = -5000;
    constexpr float MAX_OUT_LOADER_SPEED = 5000;
    static_assert(MAX_OUT_LOADER_SPEED >= MIN_OUT_LOADER_SPEED);

    // Flywheel PID.
    constexpr float KP_FLYWHEEL_SPEED = 27;
    constexpr float KI_FLYWHEEL_SPEED = 5;
    constexpr float KD_FLYWHEEL_SPEED = 0;
    constexpr float BETA_FLYWHEEL_SPEED = 1;
    constexpr float YETA_FLYWHEEL_SPEED = 0;
    constexpr float MIN_OUT_FLYWHEEL_SPEED = -5000;
    constexpr float MAX_OUT_FLYWHEEL_SPEED = 5000;
    static_assert(MAX_OUT_FLYWHEEL_SPEED >= MIN_OUT_FLYWHEEL_SPEED);
}  // namespace shoot_params

namespace chassis_params {
    // Movement targets.
    constexpr float MAX_TRANSLATION = 8;  // m/s
    constexpr float MAX_ROTATION = 5;     // rad/s
    constexpr float GYRO_SPEED = 5;       // rad/s
    static_assert(MAX_TRANSLATION >= 0);
    static_assert(MAX_ROTATION >= 0);
    static_assert(MAX_ROTATION >= GYRO_SPEED);

    // Drive wheel PID.
    constexpr float KP_DRIVE_WHEEL = 5;
    constexpr float KI_DRIVE_WHEEL = 0;
    constexpr float KD_DRIVE_WHEEL = 0;
    constexpr float BETA_DRIVE_WHEEL = 1;
    constexpr float YETA_DRIVE_WHEEL = 0;
    constexpr float MIN_OUT_DRIVE_WHEEL = -2000;
    constexpr float MAX_OUT_DRIVE_WHEEL = 2000;
    static_assert(MAX_OUT_DRIVE_WHEEL >= MIN_OUT_DRIVE_WHEEL);

    // Drive wheel ramp.
    constexpr float WHEEL_RAMP_MAX_ACCEL = 400;  // rad/s^2
    static_assert(WHEEL_RAMP_MAX_ACCEL >= 0);
}  // namespace chassis_params

#endif