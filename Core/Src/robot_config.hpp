#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

namespace robot_config {
    namespace gimbal_params {
#ifdef MECANUM_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 3350;
        constexpr int16_t PITCH_ECD_CENTER = 2035;
        constexpr float LOADER_GEAR_RATIO = 36;

        constexpr float PITCH_MIN_ANGLE = -0.25;
        constexpr float PITCH_MAX_ANGLE = 0.50;
        static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);
#endif

#ifdef OMNI_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 5750;
        constexpr int16_t PITCH_ECD_CENTER = 4800;
        constexpr float LOADER_GEAR_RATIO = 36;

        constexpr float PITCH_MIN_ANGLE = -0.14;
        constexpr float PITCH_MAX_ANGLE = 0.35;
        static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);
#endif

#ifdef HERO_GIMBAL
        constexpr float IMU_ORIENTATION[3][3] = {
            {0.0f, 1.0f, 0.0f},
            {-1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };

        constexpr int16_t YAW_ECD_CENTER = 3100;
        constexpr int16_t PITCH_ECD_CENTER = 6800;
        constexpr float LOADER_GEAR_RATIO = 3591 / 187;

        constexpr float PITCH_MIN_ANGLE = -0.3;
        constexpr float PITCH_MAX_ANGLE = 0.1;
        static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);

        constexpr float KP_GIMBAL_YAW_ANGLE = 130;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;
        constexpr float KD_GIMBAL_YAW_ANGLE = 0;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = -800;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 800;
        static_assert(MAX_OUT_GIMBAL_YAW_ANGLE >= MIN_OUT_GIMBAL_YAW_ANGLE);

        constexpr float KP_GIMBAL_YAW_SPEED = 300;
        constexpr float KI_GIMBAL_YAW_SPEED = 60;
        constexpr float KD_GIMBAL_YAW_SPEED = 0;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 20000;
        static_assert(MAX_OUT_GIMBAL_YAW_SPEED >= MIN_OUT_GIMBAL_YAW_SPEED);

        constexpr float KP_GIMBAL_PITCH_ANGLE = 180;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 40;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 1;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;
        static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

        constexpr float KP_GIMBAL_PITCH_SPEED = 240;
        constexpr float KI_GIMBAL_PITCH_SPEED = 60;
        constexpr float KD_GIMBAL_PITCH_SPEED = 0;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;
        static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
#endif

#ifdef SWERVE_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 4430;
        constexpr int16_t PITCH_ECD_CENTER = 4050;
        constexpr float LOADER_GEAR_RATIO = 36;

        constexpr float IMU_ORIENTATION[3][3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };

        constexpr float PITCH_MIN_ANGLE = -0.2;
        constexpr float PITCH_MAX_ANGLE = 0.1;
        static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);

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
#endif

#if defined(OMNI_GIMBAL) || defined(MECANUM_GIMBAL)
        // TODO: Move to front-right-up (x-y-z) reference frame???
        constexpr float IMU_ORIENTATION[3][3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f},
        };

        constexpr float KP_GIMBAL_YAW_ANGLE = 76;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;  // Not allowed.
        constexpr float KD_GIMBAL_YAW_ANGLE = 0.3;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = -800;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 800;
        static_assert(MAX_OUT_GIMBAL_YAW_ANGLE >= MIN_OUT_GIMBAL_YAW_ANGLE);

        constexpr float KP_GIMBAL_YAW_SPEED = 240;
        constexpr float KI_GIMBAL_YAW_SPEED = 10;
        constexpr float KD_GIMBAL_YAW_SPEED = 0.1;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 20000;
        static_assert(MAX_OUT_GIMBAL_YAW_SPEED >= MIN_OUT_GIMBAL_YAW_SPEED);

        constexpr float KP_GIMBAL_PITCH_ANGLE = 220;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 12;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 2;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;
        static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

        constexpr float KP_GIMBAL_PITCH_SPEED = 80;
        constexpr float KI_GIMBAL_PITCH_SPEED = 10;
        constexpr float KD_GIMBAL_PITCH_SPEED = 0.05;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;
        static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
#endif

#ifdef GTEST
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
        constexpr float LOADER_GEAR_RATIO = 1;

        constexpr float PITCH_MIN_ANGLE = 0;
        constexpr float PITCH_MAX_ANGLE = 0;
        static_assert(PITCH_MAX_ANGLE >= PITCH_MIN_ANGLE);

        constexpr float KP_GIMBAL_YAW_ANGLE = 0;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;
        constexpr float KD_GIMBAL_YAW_ANGLE = 0;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 0;
        static_assert(MAX_OUT_GIMBAL_YAW_ANGLE >= MIN_OUT_GIMBAL_YAW_ANGLE);

        constexpr float KP_GIMBAL_YAW_SPEED = 0;
        constexpr float KI_GIMBAL_YAW_SPEED = 0;
        constexpr float KD_GIMBAL_YAW_SPEED = 0;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 0;
        static_assert(MAX_OUT_GIMBAL_YAW_SPEED >= MIN_OUT_GIMBAL_YAW_SPEED);

        constexpr float KP_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 0;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 0;
        static_assert(MAX_OUT_GIMBAL_PITCH_ANGLE >= MIN_OUT_GIMBAL_PITCH_ANGLE);

        constexpr float KP_GIMBAL_PITCH_SPEED = 0;
        constexpr float KI_GIMBAL_PITCH_SPEED = 0;
        constexpr float KD_GIMBAL_PITCH_SPEED = 0;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 0;
        static_assert(MAX_OUT_GIMBAL_PITCH_SPEED >= MIN_OUT_GIMBAL_PITCH_SPEED);
#endif

        // OTHER PARAMETERS
        // How close does gimbal yaw have to be to 0 during calibration phase to
        // exit calibration?
        constexpr float EXIT_CALIBRATION_YAW_ANGLE_DELTA = 2.0;  // degrees
        static_assert(EXIT_CALIBRATION_YAW_ANGLE_DELTA >= 0);
    }  // namespace gimbal_params

    namespace shoot_params {
        // PID
#ifdef HERO_GIMBAL
        constexpr float KP_LOADER_SPEED = 15;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 1;
        constexpr float YETA_LOADER_SPEED = 0;
        constexpr float MIN_OUT_LOADER_SPEED = -5000;
        constexpr float MAX_OUT_LOADER_SPEED = 5000;
        static_assert(MAX_OUT_LOADER_SPEED >= MIN_OUT_LOADER_SPEED);

        constexpr float KP_FLYWHEEL_SPEED = 27;
        constexpr float KI_FLYWHEEL_SPEED = 5;
        constexpr float KD_FLYWHEEL_SPEED = 0;
        constexpr float BETA_FLYWHEEL_SPEED = 1;
        constexpr float YETA_FLYWHEEL_SPEED = 0;
        constexpr float MIN_OUT_FLYWHEEL_SPEED = -5000;
        constexpr float MAX_OUT_FLYWHEEL_SPEED = 5000;
        static_assert(MAX_OUT_FLYWHEEL_SPEED >= MIN_OUT_FLYWHEEL_SPEED);
#else
        constexpr float KP_LOADER_SPEED = 12;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 1;
        constexpr float YETA_LOADER_SPEED = 0;
        constexpr float MIN_OUT_LOADER_SPEED = -5000;
        constexpr float MAX_OUT_LOADER_SPEED = 5000;
        static_assert(MAX_OUT_LOADER_SPEED >= MIN_OUT_LOADER_SPEED);

        constexpr float KP_FLYWHEEL_SPEED = 27;
        constexpr float KI_FLYWHEEL_SPEED = 5;
        constexpr float KD_FLYWHEEL_SPEED = 0;
        constexpr float BETA_FLYWHEEL_SPEED = 1;
        constexpr float YETA_FLYWHEEL_SPEED = 0;
        constexpr float MIN_OUT_FLYWHEEL_SPEED = -5000;
        constexpr float MAX_OUT_FLYWHEEL_SPEED = 5000;
        static_assert(MAX_OUT_FLYWHEEL_SPEED >= MIN_OUT_FLYWHEEL_SPEED);
#endif

// OTHER PARAMETERS
#ifdef MECANUM_GIMBAL
        constexpr float LOADER_ACTIVE_RPM = 75;
        constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 7000;
        constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
        static_assert(MAX_FLYWHEEL_ACCEL >= 0);
#endif

#ifdef HERO_GIMBAL
        constexpr float LOADER_ACTIVE_RPM = 33;
        constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 10000;
        constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
        static_assert(MAX_FLYWHEEL_ACCEL >= 0);
#endif

#ifdef OMNI_GIMBAL
        constexpr float LOADER_ACTIVE_RPM = 75;
        constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 7000;
        constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
        static_assert(MAX_FLYWHEEL_ACCEL >= 0);
#endif

#ifdef SWERVE_GIMBAL
        constexpr float LOADER_ACTIVE_RPM = 75;
        constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 7000;
        constexpr float MAX_FLYWHEEL_ACCEL = 70000;  // rotations/min/second
        static_assert(MAX_FLYWHEEL_ACCEL >= 0);
#endif

        constexpr float JAM_STALL_DURATION_THRESHOLD = 0.1;  // seconds
        constexpr int16_t JAM_LOADER_RPM_THRESHOLD = 5;      // rpm
        constexpr float JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD = 0.1;
        constexpr float JAM_NO_STALL_DURATION_THRESHOLD = 0.5;  // seconds
        static_assert(JAM_STALL_DURATION_THRESHOLD >= 0);
        static_assert(JAM_LOADER_RPM_THRESHOLD >= 0);
        static_assert(JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD >= 0);
        static_assert(JAM_NO_STALL_DURATION_THRESHOLD >= 0);
    }  // namespace shoot_params

    namespace chassis_params {
        constexpr float KP_SWERVE_DRIVE = 5;
        constexpr float KI_SWERVE_DRIVE = 0;
        constexpr float KD_SWERVE_DRIVE = 0;
        constexpr float BETA_SWERVE_DRIVE = 1;
        constexpr float YETA_SWERVE_DRIVE = 0;
        constexpr float MIN_OUT_SWERVE_DRIVE = -2000;
        constexpr float MAX_OUT_SWERVE_DRIVE = 2000;
        static_assert(MAX_OUT_SWERVE_DRIVE >= MIN_OUT_SWERVE_DRIVE);

        constexpr float KP_OMNI_DRIVE = 5;
        constexpr float KI_OMNI_DRIVE = 0;
        constexpr float KD_OMNI_DRIVE = 0;
        constexpr float BETA_OMNI_DRIVE = 1;
        constexpr float YETA_OMNI_DRIVE = 0;
        constexpr float MIN_OUT_OMNI_DRIVE = -5000;
        constexpr float MAX_OUT_OMNI_DRIVE = 5000;
        static_assert(MAX_OUT_OMNI_DRIVE >= MIN_OUT_OMNI_DRIVE);

        constexpr float WHEEL_RAMP_MAX_ACCEL = 400;  // rad/s^2
        static_assert(WHEEL_RAMP_MAX_ACCEL >= 0);

        constexpr float KP_CHASSIS_SPIN = 1;
        constexpr float KI_CHASSIS_SPIN = 0;
        constexpr float KD_CHASSIS_SPIN = 0;
        constexpr float BETA_CHASSIS_SPIN = 1;
        constexpr float YETA_CHASSIS_SPIN = 0;
        constexpr float MIN_OUT_CHASSIS_SPIN = -2;
        constexpr float MAX_OUT_CHASSIS_SPIN = 2;
        static_assert(MAX_OUT_OMNI_DRIVE >= MIN_OUT_OMNI_DRIVE);

        constexpr float MAX_TRANSLATION = 4;      // m/s
        constexpr float MAX_ROTATION = 3.141527;  // rad/s
        constexpr float GYRO_SPEED = 3.141527;    // rad/s
        static_assert(MAX_TRANSLATION >= 0);
        static_assert(MAX_ROTATION >= 0);
        static_assert(MAX_ROTATION >= GYRO_SPEED);
    }  // namespace chassis_params
}  // namespace robot_config

#endif
