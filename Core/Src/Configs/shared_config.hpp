#ifndef __SHARED_CONFIG_HPP
#define __SHARED_CONFIG_HPP

#include <cstdint>

namespace gimbal_params {
    constexpr float EXIT_CALIBRATION_YAW_ANGLE_DELTA = 2.0;  // degrees
    static_assert(EXIT_CALIBRATION_YAW_ANGLE_DELTA >= 0);
}  // namespace gimbal_params

namespace shoot_params {
    // Anti-jam.
    constexpr float JAM_STALL_DURATION_THRESHOLD = 0.5;  // seconds
    constexpr int16_t JAM_LOADER_RPM_THRESHOLD = 5;      // rpm
    constexpr float JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD = 0.05;
    constexpr float JAM_NO_STALL_DURATION_THRESHOLD = 0.5;  // seconds
    static_assert(JAM_STALL_DURATION_THRESHOLD >= 0);
    static_assert(JAM_LOADER_RPM_THRESHOLD >= 0);
    static_assert(JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD >= 0);
    static_assert(JAM_NO_STALL_DURATION_THRESHOLD >= 0);

}  // namespace shoot_params

namespace chassis_params {
    // Gimbal Center Spin PID.
    constexpr float KP_CHASSIS_SPIN = 1;
    constexpr float KI_CHASSIS_SPIN = 0;
    constexpr float KD_CHASSIS_SPIN = 0;
    constexpr float BETA_CHASSIS_SPIN = 1;
    constexpr float YETA_CHASSIS_SPIN = 0;
    constexpr float MIN_OUT_CHASSIS_SPIN = -2;
    constexpr float MAX_OUT_CHASSIS_SPIN = 2;
    static_assert(MAX_OUT_CHASSIS_SPIN >= MIN_OUT_CHASSIS_SPIN);
}  // namespace chassis_params

#endif