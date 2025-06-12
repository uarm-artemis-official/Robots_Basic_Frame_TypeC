#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

namespace robot_config {
    namespace gimbal_params {
#ifdef MECANUM_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 3350;
        constexpr int16_t PITCH_ECD_CENTER = 2035;
#endif

#ifdef OMNI_GIMBAL
        constexpr int16_t YAW_ECD_CENTER = 1025;
        constexpr int16_t PITCH_ECD_CENTER = 4800;
#endif

#ifdef HERO
        static_assert(false, "Uncalibrated Gimbal encoder centers.");
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
#endif

#ifdef SWERVE_GIMBAL
        static_assert(false, "Uncalibrated Gimbal encoder centers.");
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
#endif

#ifdef GTEST
        constexpr int16_t YAW_ECD_CENTER = 0;
        constexpr int16_t PITCH_ECD_CENTER = 0;
#endif
    }  // namespace gimbal_params

    namespace pid_params {
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

#ifdef TUNE_GIMBAL_PID
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
#else
/* gimbal 6020 dual loop control */
#ifdef HERO_GIMBAL
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
#else
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

#ifdef HERO_GIMBAL
        constexpr float KP_LOADER_SPEED = 0;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 1;
        constexpr float YETA_LOADER_SPEED = 1;
        constexpr float MIN_OUT_LOADER_SPEED = -1000;
        constexpr float MAX_OUT_LOADER_SPEED = 1000;
#else
        constexpr float KP_LOADER_SPEED = 5;
        constexpr float KI_LOADER_SPEED = 0;
        constexpr float KD_LOADER_SPEED = 0;
        constexpr float BETA_LOADER_SPEED = 1;
        constexpr float YETA_LOADER_SPEED = 0;
        constexpr float MIN_OUT_LOADER_SPEED = -1000;
        constexpr float MAX_OUT_LOADER_SPEED = 1000;
#endif
        constexpr float KP_FLYWHEEL_SPEED = 20;
        constexpr float KI_FLYWHEEL_SPEED = 0.1;
        constexpr float KD_FLYWHEEL_SPEED = 0.3;
        constexpr float BETA_FLYWHEEL_SPEED = 1;
        constexpr float YETA_FLYWHEEL_SPEED = 0;
        constexpr float MIN_OUT_FLYWHEEL_SPEED = -1000;
        constexpr float MAX_OUT_FLYWHEEL_SPEED = 1000;
#endif
    }  // namespace pid_params
}  // namespace robot_config

#endif