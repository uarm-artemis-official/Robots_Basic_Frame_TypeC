/*******************************************************************************
* @file           : Chassis_App.c
* @brief          : chassis task managing 4 chassis motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
//#define CHASSIS_POWER_LIMIT

#include "Chassis_App.h"
#include <cstring>
#include <type_traits>
#include "Omni_Drive.h"
#include "Swerve_Drive.h"
#include "apps_defines.h"
#include "pid.h"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

template class ChassisApp<OmniDrive>;
template class ChassisApp<SwerveDrive>;

template <class DriveTrain>
ChassisApp<DriveTrain>::ChassisApp(DriveTrain& drive_train_ref,
                                   IMessageCenter& message_center_ref,
                                   IDebug& debug_ref)
    : drive_train(drive_train_ref),
      message_center(message_center_ref),
      debug(debug_ref) {
    static_assert(
        std::is_same<DriveTrain, OmniDrive>::value ||
            std::is_same<DriveTrain, SwerveDrive>::value,
        "Attempt to initialize ChassisApp with unsupported DriveTrain.");
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::init() {
    drive_train.init();
    debug.set_led_state(RED, ON);

    pid2_init(chassis.spin_pid, robot_config::chassis_params::KP_CHASSIS_SPIN,
              robot_config::chassis_params::KI_CHASSIS_SPIN,
              robot_config::chassis_params::KD_CHASSIS_SPIN,
              robot_config::chassis_params::BETA_CHASSIS_SPIN,
              robot_config::chassis_params::YETA_CHASSIS_SPIN,
              -ChassisApp::MAX_ROTATION, ChassisApp::MAX_ROTATION);

    /* set initial chassis mode to idle mode or debug mode */
    chassis.chassis_mode = IDLE_MODE;
    chassis.chassis_act_mode = INDPET_MODE;
    chassis.chassis_gear_mode = AUTO_GEAR;

    set_initial_state();
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::set_initial_state() {
    chassis.vx = 0;
    chassis.vy = 0;
    chassis.wz = 0;
    chassis.max_vx = robot_config::chassis_params::MAX_TRANSLATION;
    chassis.max_vy = robot_config::chassis_params::MAX_TRANSLATION;
    chassis.max_wz = robot_config::chassis_params::MAX_ROTATION;

    chassis.v_perp = 0;
    chassis.v_parallel = 0;

    chassis.gimbal_yaw_rel_angle = 0;

    std::memset(&(chassis.ref_power_stat), 0, sizeof(ChassisPowerStat_t));
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::loop() {
    chassis_get_gimbal_rel_angles();

    process_commands();

    calc_movement_vectors();

    drive_train.drive(chassis.vx, chassis.vy, chassis.wz);
}

/**
 * Calculate inverse kinematics based on Chassis Act Mode.
 * (INDPET_MODE) Chassis and gimbal are aligned and share the same reference frame.
 * (GIMBAL_CENTER) Gimbal can move freely and chassis tries to orientate itself with 
 * gimbal's reference frame.
 * (GYRO_MODE) Chassis continuously spins and moves along gimbal's reference frame.
 */
template <class DriveTrain>
void ChassisApp<DriveTrain>::calc_movement_vectors() {
    if (chassis.chassis_mode == IDLE_MODE) {
        chassis.vx = 0;
        chassis.vy = 0;
        chassis.wz = 0;
        return;
    }

    // Inverse kinematics requires gimbal yaw angle needs to be adjusted to principal axis.
    switch (chassis.chassis_act_mode) {
        case INDPET_MODE:
            chassis.vx = chassis.v_perp;
            chassis.vy = chassis.v_parallel;
            break;
        case GIMBAL_CENTER:
            chassis.vx = chassis.v_perp;
            chassis.vy = chassis.v_parallel;
            chassis.wz = pid2_single_loop_control(
                chassis.spin_pid, 0, chassis.gimbal_yaw_rel_angle,
                ChassisApp::LOOP_PERIOD_MS * 0.001);
            break;
        case SELF_GYRO:
            chassis.vx =
                chassis.v_perp * arm_cos_f32(chassis.gimbal_yaw_rel_angle) -
                chassis.v_parallel * arm_sin_f32(chassis.gimbal_yaw_rel_angle);
            chassis.vy =
                chassis.v_perp * arm_sin_f32(chassis.gimbal_yaw_rel_angle) +
                chassis.v_parallel * arm_cos_f32(chassis.gimbal_yaw_rel_angle);

            if (fabs(chassis.v_perp) > 0.1 || fabs(chassis.v_parallel) > 0.1) {
                chassis.wz = robot_config::chassis_params::GYRO_SPEED / 2;
            } else {
                chassis.wz = robot_config::chassis_params::GYRO_SPEED;
            }
            break;
        default:
            chassis.vx = 0;
            chassis.vy = 0;
            chassis.wz = 0;
    }

    chassis.vx = value_limit(chassis.vx, -chassis.max_vx, chassis.max_vx);
    chassis.vy = value_limit(chassis.vy, -chassis.max_vy, chassis.max_vy);
    chassis.wz = value_limit(chassis.wz, -chassis.max_wz, chassis.max_wz);
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::chassis_get_gimbal_rel_angles() {
    float rel_angles[2];
    BaseType_t new_rel_angle_message =
        message_center.peek_message(GIMBAL_REL_ANGLES, rel_angles, 0);
    if (new_rel_angle_message == pdTRUE) {
        chassis.gimbal_yaw_rel_angle = rel_angles[0];
    }
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::process_commands() {
    ChassisCommandMessage_t chassis_command;
    uint8_t new_message =
        message_center.get_message(COMMAND_CHASSIS, &chassis_command, 0);
    if (new_message == pdTRUE) {
        chassis.v_perp = chassis_command.v_perp;
        chassis.v_parallel = chassis_command.v_parallel;
        chassis.wz = chassis_command.wz;

        BoardMode_t board_mode =
            static_cast<BoardMode_t>((chassis_command.command_bits >> 3) & 0x7);
        BoardActMode_t act_mode =
            static_cast<BoardActMode_t>(chassis_command.command_bits & 0x7);

        set_board_mode(board_mode);
        set_act_mode(act_mode);
    }
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::set_board_mode(BoardMode_t new_board_mode) {
    switch (new_board_mode) {
        case PATROL_MODE:
        case AUTO_AIM_MODE:
        case AUTO_PILOT_MODE:  // full control to mini-pc.
        case IDLE_MODE:
            chassis.chassis_mode = new_board_mode;
            break;
        default:
            return;
    }
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::set_act_mode(BoardActMode_t new_act_mode) {
    switch (new_act_mode) {
        case GIMBAL_CENTER:
        case GIMBAL_FOLLOW:
        case SELF_GYRO:
        case INDPET_MODE:
            chassis.chassis_act_mode = new_act_mode;
            break;
        default:
            return;
    }
}
