#ifndef __APPS_CONFIG_H
#define __APPS_CONFIG_H

#include <cstdint>

/* =========================================================================
 * PUBLIC DEFINES 
 * ====================================================================== */
/* Task exec time in milliseconds */
namespace apps_defines {
    constexpr uint32_t chassis_task_loop_period_ms = 5;
    constexpr uint32_t gimbal_task_loop_period_ms = 5;
    constexpr uint32_t shoot_task_loop_period_ms = 5;
    constexpr uint32_t imu_task_loop_period_ms = 2;
    constexpr uint32_t timer_task_loop_period_ms = 1;
    constexpr uint32_t comm_task_loop_period_ms = 2;
    constexpr uint32_t rc_task_loop_period_ms = 2;
    constexpr uint32_t pc_uart_task_loop_period_ms = 1;
    constexpr uint32_t referee_task_loop_period_ms = 10;

    namespace chassis {
        constexpr float omni_wheel_radius = 0.076;  // meters
        constexpr float wheel_motor_reduction_ratio = 3591 / 187;
        constexpr float motor_max_output = 16384;
    }  // namespace chassis

    namespace gimbal {}

    namespace rc {
        constexpr float mouse_max_yaw_magnitude_out = 2;
        constexpr float mouse_max_pitch_magnitude_out = 2;
        constexpr int16_t joystick_max_offset_magnitude = 660;

        constexpr float gimbal_joystick_send_threshold = 0.001;
        constexpr float chassis_joystick_send_threshold = 0.01;
    }  // namespace rc

    namespace referee {}
}  // namespace apps_defines

/* =========================================================================
 * GIMBAL DEFINES
 * ====================================================================== */
#define GIMBAL_YAW_MOTOR_INDEX 0
#define GIMBAL_PITCH_MOTOR_INDEX 1
#define GIMBAL_MOTOR_COUNT 2

/* =========================================================================
 * TIMER DEFINES
 * ====================================================================== */
// #define DISABLE_MOTOR_SEND

/* =========================================================================
 * REFEREE DEFINES
 * ====================================================================== */
#define REFEREE_NON_RECV_MAX_COUNT \
    100  // maximum count of non-received referee data before reset
// now is 1 second

#endif