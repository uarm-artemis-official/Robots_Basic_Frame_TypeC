#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

namespace robot_config {
    namespace gimbal_params {
#ifdef MECANUM_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 3350;
        constexpr int16_t PITCH_ECD_CENTER = 2035;
        constexpr uint32_t LOADER_GEAR_RATIO = 36;
#endif

#ifdef OMNI_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 1025;
        constexpr int16_t PITCH_ECD_CENTER = 4800;
        constexpr uint32_t LOADER_GEAR_RATIO = 36;
#endif

#ifdef HERO
        static_assert(false, "Uncalibrated Gimbal encoder centers.");
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
        constexpr uint32_t LOADER_GEAR_RATIO = 1;

        constexpr float KP_GIMBAL_YAW_ANGLE = 0;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;
        constexpr float KD_GIMBAL_YAW_ANGLE = 0;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 0;

        constexpr float KP_GIMBAL_YAW_SPEED = 0;
        constexpr float KI_GIMBAL_YAW_SPEED = 0;
        constexpr float KD_GIMBAL_YAW_SPEED = 0;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 0;

        constexpr float KP_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 0;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 0;

        constexpr float KP_GIMBAL_PITCH_SPEED = 0;
        constexpr float KI_GIMBAL_PITCH_SPEED = 0;
        constexpr float KD_GIMBAL_PITCH_SPPED = 0;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 0;
#endif

#ifdef SWERVE_GIMBAL
        static_assert(false, "Uncalibrated Gimbal encoder centers.");
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
        constexpr uint32_t LOADER_GEAR_RATIO = 36;
#endif

#if defined(SWERVE_GIMBAL) || defined(OMNI_GIMBAL) || defined(MECANUM_GIMBAL)
        constexpr float KP_GIMBAL_YAW_ANGLE = 150;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;
        constexpr float KD_GIMBAL_YAW_ANGLE = 0;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = -800;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 800;

        constexpr float KP_GIMBAL_YAW_SPEED = 400;
        constexpr float KI_GIMBAL_YAW_SPEED = 70;
        constexpr float KD_GIMBAL_YAW_SPEED = 0;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 20000;

        constexpr float KP_GIMBAL_PITCH_ANGLE = 300;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 30;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 0;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = -1000;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 1000;

        constexpr float KP_GIMBAL_PITCH_SPEED = 180;
        constexpr float KI_GIMBAL_PITCH_SPEED = 0;
        constexpr float KD_GIMBAL_PITCH_SPEED = 0;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 0;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = -20000;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 20000;
#endif

#ifdef GTEST
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
        constexpr uint32_t LOADER_GEAR_RATIO = 1;

        constexpr float KP_GIMBAL_YAW_ANGLE = 0;
        constexpr float KI_GIMBAL_YAW_ANGLE = 0;
        constexpr float KD_GIMBAL_YAW_ANGLE = 0;
        constexpr float BETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float YETA_GIMBAL_YAW_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_ANGLE = 0;

        constexpr float KP_GIMBAL_YAW_SPEED = 0;
        constexpr float KI_GIMBAL_YAW_SPEED = 0;
        constexpr float KD_GIMBAL_YAW_SPEED = 0;
        constexpr float BETA_GIMBAL_YAW_SPEED = 1;
        constexpr float YETA_GIMBAL_YAW_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_YAW_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_YAW_SPEED = 0;

        constexpr float KP_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KI_GIMBAL_PITCH_ANGLE = 0;
        constexpr float KD_GIMBAL_PITCH_ANGLE = 0;
        constexpr float BETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float YETA_GIMBAL_PITCH_ANGLE = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_ANGLE = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_ANGLE = 0;

        constexpr float KP_GIMBAL_PITCH_SPEED = 0;
        constexpr float KI_GIMBAL_PITCH_SPEED = 0;
        constexpr float KD_GIMBAL_PITCH_SPPED = 0;
        constexpr float BETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float YETA_GIMBAL_PITCH_SPEED = 1;
        constexpr float MIN_OUT_GIMBAL_PITCH_SPEED = 0;
        constexpr float MAX_OUT_GIMBAL_PITCH_SPEED = 0;
#endif

        // OTHER PARAMETERS
        // How close does gimbal yaw have to be to 0 during calibration phase to
        // exit calibration?
        constexpr float EXIT_CALIBRATION_YAW_ANGLE_DELTA = 2.0;  // degrees

        // (WIP) Software lockout limits on pitch target to prevent hitting mechanical
        // hard-stops.
        // Important for PID to remain in controllable linear region. Hard-stops
        // are non-linear uncontrollable regions.
        // TODO: Implement pitch software locks.
        constexpr float LOCKOUT_BOTTOM_PITCH_ANGLE = 0;
        constexpr float LOCKOUT_TOP_PITCH_ANGLE = 0;
    }  // namespace gimbal_params

    namespace shoot_params {
        // PID
#ifdef HERO_GIMBAL
        constexpr float KP_LOADER_SPEED = 0;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 0;
        constexpr float YETA_LOADER_SPEED = 0;
        constexpr float MIN_OUT_LOADER_SPEED = 0;
        constexpr float MAX_OUT_LOADER_SPEED = 0;

        constexpr float KP_FLYWHEEL_SPEED = 0;
        constexpr float KI_FLYWHEEL_SPEED = 0;
        constexpr float KD_FLYWHEEL_SPEED = 0;
        constexpr float BETA_FLYWHEEL_SPEED = 0;
        constexpr float YETA_FLYWHEEL_SPEED = 0;
        constexpr float MIN_OUT_FLYWHEEL_SPEED = 0;
        constexpr float MAX_OUT_FLYWHEEL_SPEED = 0;
#else
        constexpr float KP_LOADER_SPEED = 5;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 1;
        constexpr float YETA_LOADER_SPEED = 0;
        constexpr float MIN_OUT_LOADER_SPEED = -5000;
        constexpr float MAX_OUT_LOADER_SPEED = 5000;

        constexpr float KP_FLYWHEEL_SPEED = 25;
        constexpr float KI_FLYWHEEL_SPEED = 15;
        constexpr float KD_FLYWHEEL_SPEED = 0;
        constexpr float BETA_FLYWHEEL_SPEED = 1;
        constexpr float YETA_FLYWHEEL_SPEED = 0;
        constexpr float MIN_OUT_FLYWHEEL_SPEED = -5000;
        constexpr float MAX_OUT_FLYWHEEL_SPEED = 5000;
#endif

        // OTHER PARAMETERS
        constexpr float LOADER_ACTIVE_RPM = 10;
        constexpr float FLYWHEEL_ACTIVE_TARGET_RPM = 100;
        constexpr float MAX_FLYWHEEL_ACCEL = 100;  // rotations/min/second

        constexpr float JAM_STALL_DURATION_THRESHOLD = 0.1;  // seconds
        constexpr int16_t JAM_LOADER_RPM_THRESHOLD = 5;      // rpm
        constexpr float JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD = 0.1;
        constexpr float JAM_NO_STALL_DURATION_THRESHOLD = 0.5;  // seconds
    }  // namespace shoot_params

    namespace chassis_params {
        constexpr float KP_SWERVE_DRIVE = 5;
        constexpr float KI_SWERVE_DRIVE = 0;
        constexpr float KD_SWERVE_DRIVE = 0;
        constexpr float BETA_SWERVE_DRIVE = 1;
        constexpr float YETA_SWERVE_DRIVE = 0;
        constexpr float MIN_OUT_SWERVE_DRIVE = -2000;
        constexpr float MAX_OUT_SWERVE_DRIVE = 2000;

        constexpr float KP_OMNI_DRIVE = 5;
        constexpr float KI_OMNI_DRIVE = 0;
        constexpr float KD_OMNI_DRIVE = 0;
        constexpr float BETA_OMNI_DRIVE = 1;
        constexpr float YETA_OMNI_DRIVE = 0;
        constexpr float MIN_OUT_OMNI_DRIVE = -5000;
        constexpr float MAX_OUT_OMNI_DRIVE = 5000;

        constexpr float KP_CHASSIS_SPIN = 1;
        constexpr float KI_CHASSIS_SPIN = 0;
        constexpr float KD_CHASSIS_SPIN = 0;
        constexpr float BETA_CHASSIS_SPIN = 1;
        constexpr float YETA_CHASSIS_SPIN = 0;
        constexpr float MIN_OUT_CHASSIS_SPIN = -2;
        constexpr float MAX_OUT_CHASSIS_SPIN = 2;
    }  // namespace chassis_params
}  // namespace robot_config

#endif