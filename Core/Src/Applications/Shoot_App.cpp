/*******************************************************************************
* @file           : Shoot_App.c
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include "Shoot_App.h"
#include <cstring>
#include "apps_defines.h"
#include "pid.h"
#include "robot_config.hpp"

// static uint8_t shoot_reserve_flag = 0;
// static uint8_t shoot_reserve_counter = 0;
// static uint8_t shoot_check_flag = 0;
// static uint16_t shoot_check_counter = 0;

// TODO: Re-implement reverse/unjamming mechanism.
// Possible use torque current feedback to detect jam.

namespace {
    constexpr size_t LEFT_FLYWHEEL_INDEX = 0;
    constexpr size_t RIGHT_FLYWHEEL_INDEX = 1;
}  // namespace

ShootApp::ShootApp(IMessageCenter& message_center_ref, IAmmoLid& ammo_lid_ref,
                   IMotors& motors_ref, float loader_rpm_)
    : message_center(message_center_ref),
      ammo_lid(ammo_lid_ref),
      motors(motors_ref),
      loader_rpm(loader_rpm_) {}

void ShootApp::init() {
    ammo_lid.init();

    loader_control.stdid = SHOOT_LOADER;

    // TODO: Add configuration for hero loader.
    pid2_init(loader_control.speed_pid,
              robot_config::pid_params::KP_LOADER_SPEED,
              robot_config::pid_params::KI_LOADER_SPEED,
              robot_config::pid_params::KD_LOADER_SPEED,
              robot_config::pid_params::BETA_LOADER_SPEED,
              robot_config::pid_params::YETA_LOADER_SPEED,
              robot_config::pid_params::MIN_OUT_LOADER_SPEED,
              robot_config::pid_params::MAX_OUT_LOADER_SPEED);
    std::memset(&(loader_control.feedback), 0, sizeof(Motor_Feedback_t));

    flywheel_controls[LEFT_FLYWHEEL_INDEX].stdid = SHOOT_LEFT_FRIC;
    pid2_init(flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid,
              robot_config::pid_params::KP_FLYWHEEL_SPEED,
              robot_config::pid_params::KI_FLYWHEEL_SPEED,
              robot_config::pid_params::KD_FLYWHEEL_SPEED,
              robot_config::pid_params::BETA_FLYWHEEL_SPEED,
              robot_config::pid_params::YETA_FLYWHEEL_SPEED,
              robot_config::pid_params::MIN_OUT_FLYWHEEL_SPEED,
              robot_config::pid_params::MAX_OUT_FLYWHEEL_SPEED);
    std::memset(&(flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback), 0,
                sizeof(Motor_Feedback_t));

    flywheel_controls[RIGHT_FLYWHEEL_INDEX].stdid = SHOOT_RIGHT_FRIC;
    pid2_init(flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid,
              robot_config::pid_params::KP_FLYWHEEL_SPEED,
              robot_config::pid_params::KI_FLYWHEEL_SPEED,
              robot_config::pid_params::KD_FLYWHEEL_SPEED,
              robot_config::pid_params::BETA_FLYWHEEL_SPEED,
              robot_config::pid_params::YETA_FLYWHEEL_SPEED,
              robot_config::pid_params::MIN_OUT_FLYWHEEL_SPEED,
              robot_config::pid_params::MAX_OUT_FLYWHEEL_SPEED);
    std::memset(&(flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback), 0,
                sizeof(Motor_Feedback_t));

    std::memset(&(shoot), 0, sizeof(Shoot));

    /* set shoot mode */
    set_shoot_mode(SHOOT_CEASE);
    ammo_lid.set_lid_status(EAmmoLidStatus::CLOSED);
}

void ShootApp::loop() {
    get_rc_info();
    get_motor_feedback();

    calc_targets();
    calc_motor_outputs();

    send_motor_outputs();
}

void ShootApp::get_rc_info() {
    RCInfoMessage_t rc_info;
    BaseType_t new_rc_info_message =
        message_center.peek_message(RC_INFO, &rc_info, 0);
    if (new_rc_info_message == pdTRUE) {
        ShootActMode_t shoot_mode =
            static_cast<ShootActMode_t>(rc_info.modes[2]);
        set_shoot_mode(shoot_mode);
    }
}

void ShootApp::get_motor_feedback() {
    MotorReadMessage_t read_message;
    BaseType_t has_motor_feedback =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (has_motor_feedback == pdTRUE) {
        std::array<std::pair<uint32_t, Motor_Feedback_t*>, 3> feedbacks = {
            std::make_pair(loader_control.stdid, &(loader_control.feedback)),
            std::make_pair(flywheel_controls[LEFT_FLYWHEEL_INDEX].stdid,
                           &(flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback)),
            std::make_pair(flywheel_controls[RIGHT_FLYWHEEL_INDEX].stdid,
                           &(flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback)),
        };

        for (size_t i = 0; i < feedbacks.size(); i++) {
            for (size_t j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (read_message.can_ids[j] == feedbacks[i].first) {
                    motors.get_raw_feedback(feedbacks[i].first,
                                            read_message.feedback[j],
                                            feedbacks[i].second);
                    break;
                }
            }
        }
    }
}

void ShootApp::set_shoot_mode(ShootActMode_t new_mode) {
    shoot.shoot_act_mode = new_mode;
}

void ShootApp::calc_targets() {
    switch (shoot.shoot_act_mode) {
        case SHOOT_CEASE:
            shoot.flywheel_target_rpm = 0;
            shoot.loader_target_rpm = 0;
            break;
        case SHOOT_CONT:
            shoot.flywheel_target_rpm = FLYWHEEL_TARGET_RPM;
            shoot.loader_target_rpm = loader_rpm;
            break;
        default:
            shoot.flywheel_target_rpm = 0;
            shoot.loader_target_rpm = 0;
    }
}

void ShootApp::calc_motor_outputs() {
    // TODO: Loader calculations.

    pid2_single_loop_control(
        flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid,
        -shoot.flywheel_target_rpm,
        flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
        LOOP_PERIOD_MS * 0.001);

    pid2_single_loop_control(
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid,
        shoot.flywheel_target_rpm,
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
        LOOP_PERIOD_MS * 0.001);

    pid2_single_loop_control(loader_control.speed_pid, shoot.loader_target_rpm,
                             loader_control.feedback.rx_rpm,
                             LOOP_PERIOD_MS * 0.001);
}

void ShootApp::send_motor_outputs() {
    MotorSetMessage_t motor_set_message;
    memset(&motor_set_message, 0, sizeof(MotorSetMessage_t));

    motor_set_message.motor_can_volts[0] = loader_control.speed_pid.total_out;
    motor_set_message.can_ids[0] = loader_control.stdid;
    motor_set_message.motor_can_volts[1] =
        flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid.total_out;
    motor_set_message.can_ids[1] = flywheel_controls[LEFT_FLYWHEEL_INDEX].stdid;
    motor_set_message.motor_can_volts[2] =
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid.total_out;
    motor_set_message.can_ids[2] =
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].stdid;

    message_center.pub_message(MOTOR_SET, &motor_set_message);
}
