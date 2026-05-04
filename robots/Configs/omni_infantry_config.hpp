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
    constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
    constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;
    static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
    static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
}  // namespace gimbal_params

namespace shoot_params {
    // Shoot targets.
    constexpr float LOADER_ACTIVE_RPM = 75;
    constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 7000;
    constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
    static_assert(MAX_FLYWHEEL_ACCEL >= 0);

    // Loader PID.
    constexpr float KP_LOADER_SPEED = 30;
    constexpr float KI_LOADER_SPEED = 0;
    constexpr float KD_LOADER_SPEED = 0;
    constexpr float BETA_LOADER_SPEED = 1;
    constexpr float YETA_LOADER_SPEED = 0;
    constexpr float MIN_OUT_LOADER_SPEED = -10000;
    constexpr float MAX_OUT_LOADER_SPEED = 10000;
    static_assert(MAX_OUT_LOADER_SPEED >= MIN_OUT_LOADER_SPEED);

    // Loader PID.
    constexpr float KP_LOADER_POSITION = 0;
    constexpr float KI_LOADER_POSITION = 0;
    constexpr float KD_LOADER_POSITION = 0;
    constexpr float BETA_LOADER_POSITION = 1;
    constexpr float YETA_LOADER_POSITION = 0;
    constexpr float MIN_OUT_LOADER_POSITION = -10000;
    constexpr float MAX_OUT_LOADER_POSITION = 10000;
    static_assert(MAX_OUT_LOADER_POSITION >= MIN_OUT_LOADER_POSITION);

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
    constexpr float MIN_OUT_DRIVE_WHEEL = -5000;
    constexpr float MAX_OUT_DRIVE_WHEEL = 5000;
    static_assert(MAX_OUT_DRIVE_WHEEL >= MIN_OUT_DRIVE_WHEEL);

    // Drive wheel ramp.
    constexpr float WHEEL_RAMP_MAX_ACCEL = 400;  // rad/s^2
    static_assert(WHEEL_RAMP_MAX_ACCEL >= 0);
}  // namespace chassis_params

#endif