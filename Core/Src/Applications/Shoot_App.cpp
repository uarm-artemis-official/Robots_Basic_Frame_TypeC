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
#include "ramp.hpp"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"

namespace {
    constexpr size_t LEFT_FLYWHEEL_INDEX = 0;
    constexpr size_t RIGHT_FLYWHEEL_INDEX = 1;
}  // namespace

ShootApp::ShootApp(IMessageCenter& message_center_ref, IAmmoLid& ammo_lid_ref,
                   IMotors& motors_ref, float loader_active_rpm_,
                   float flywheel_target_rpm_, float max_flywheel_accel)
    : message_center(message_center_ref),
      ammo_lid(ammo_lid_ref),
      motors(motors_ref),
      LOADER_ACTIVE_RPM(loader_active_rpm_),
      FLYWHEEL_ACTIVE_TARGET_RPM(flywheel_target_rpm_),
      MAX_FLYWHEEL_ACCEL(max_flywheel_accel) {}

void ShootApp::init() {
    ammo_lid.init();

    loader_control.stdid = SHOOT_LOADER;

    // TODO: Add configuration for hero loader.
    pid2_init(loader_control.speed_pid,
              robot_config::shoot_params::KP_LOADER_SPEED,
              robot_config::shoot_params::KI_LOADER_SPEED,
              robot_config::shoot_params::KD_LOADER_SPEED,
              robot_config::shoot_params::BETA_LOADER_SPEED,
              robot_config::shoot_params::YETA_LOADER_SPEED,
              robot_config::shoot_params::MIN_OUT_LOADER_SPEED,
              robot_config::shoot_params::MAX_OUT_LOADER_SPEED);
    std::memset(&(loader_control.feedback), 0, sizeof(Motor_Feedback_t));

    flywheel_controls[LEFT_FLYWHEEL_INDEX].stdid = SHOOT_LEFT_FRIC;
    pid2_init(flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid,
              robot_config::shoot_params::KP_FLYWHEEL_SPEED,
              robot_config::shoot_params::KI_FLYWHEEL_SPEED,
              robot_config::shoot_params::KD_FLYWHEEL_SPEED,
              robot_config::shoot_params::BETA_FLYWHEEL_SPEED,
              robot_config::shoot_params::YETA_FLYWHEEL_SPEED,
              robot_config::shoot_params::MIN_OUT_FLYWHEEL_SPEED,
              robot_config::shoot_params::MAX_OUT_FLYWHEEL_SPEED);
    ramp_init(flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp,
              MAX_FLYWHEEL_ACCEL);
    std::memset(&(flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback), 0,
                sizeof(Motor_Feedback_t));

    flywheel_controls[RIGHT_FLYWHEEL_INDEX].stdid = SHOOT_RIGHT_FRIC;
    pid2_init(flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid,
              robot_config::shoot_params::KP_FLYWHEEL_SPEED,
              robot_config::shoot_params::KI_FLYWHEEL_SPEED,
              robot_config::shoot_params::KD_FLYWHEEL_SPEED,
              robot_config::shoot_params::BETA_FLYWHEEL_SPEED,
              robot_config::shoot_params::YETA_FLYWHEEL_SPEED,
              robot_config::shoot_params::MIN_OUT_FLYWHEEL_SPEED,
              robot_config::shoot_params::MAX_OUT_FLYWHEEL_SPEED);
    ramp_init(flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
              MAX_FLYWHEEL_ACCEL);
    std::memset(&(flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback), 0,
                sizeof(Motor_Feedback_t));

    std::memset(&(shoot), 0, sizeof(Shoot));
    shoot.shoot_state = ShootState::NORMAL;
    shoot.antijam_direction = 1;

    /* set shoot mode */
    set_shoot_mode(SHOOT_CEASE);
    ammo_lid.set_lid_status(EAmmoLidStatus::CLOSED);
}

void ShootApp::loop() {
    process_commands();
    get_motor_feedback();
    detect_loader_stall();

    calc_targets();
    calc_motor_outputs();

    send_motor_outputs();
}

void ShootApp::process_commands() {
    ShootCommandMessage_t shoot_command;
    uint8_t new_message =
        message_center.get_message(COMMAND_SHOOT, &shoot_command, 0);

    if (new_message == pdTRUE) {
        ShootActMode_t shoot_mode =
            static_cast<ShootActMode_t>(shoot_command.command_bits);
        set_shoot_mode(shoot_mode);

        uint8_t open_ammo_lid = static_cast<uint8_t>(shoot_command.extra_bits);
        if (open_ammo_lid == 1) {
            ammo_lid.set_lid_status(EAmmoLidStatus::OPEN);
        } else {
            ammo_lid.set_lid_status(EAmmoLidStatus::CLOSED);
        }
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

void ShootApp::detect_loader_stall() {
    int16_t current_loader_rpm = loader_control.feedback.rx_rpm;
    float current_loader_current = loader_control.speed_pid.total_out;
    float current_loader_output = loader_control.speed_pid.total_out;

    if (fabs(current_loader_output) > 0 &&
        abs(current_loader_rpm) <
            robot_config::shoot_params::JAM_LOADER_RPM_THRESHOLD &&
        fabs(current_loader_current - current_loader_output) /
                current_loader_current <
            robot_config::shoot_params::
                JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD) {
        shoot.stall_duration += LOOP_PERIOD_MS * 0.001;
    } else {
        shoot.stall_duration = 0;
        shoot.no_stall_duration += LOOP_PERIOD_MS * 0.001;
    }

    if (shoot.stall_duration >
        robot_config::shoot_params::JAM_STALL_DURATION_THRESHOLD) {
        shoot.shoot_state = ShootState::ANTIJAM;
        shoot.antijam_direction *= -1;
        shoot.no_stall_duration = 0;
    }

    if (shoot.no_stall_duration >
        robot_config::shoot_params::JAM_NO_STALL_DURATION_THRESHOLD) {
        shoot.shoot_state = ShootState::NORMAL;
    }
}

void ShootApp::calc_targets() {
    switch (shoot.shoot_act_mode) {
        case SHOOT_CEASE:
            shoot.shoot_state = ShootState::NORMAL;
            shoot.antijam_direction = -1;
            set_loader_target(0);
            set_flywheel_target(0);
            shoot.loader_delay_counter = 0;
            break;
        case SHOOT_CONT:
            shoot.loader_delay_counter =
                value_limit(shoot.loader_delay_counter + 1, 0, 1000);
            if (shoot.shoot_state == ShootState::NORMAL) {
                if (shoot.loader_delay_counter >= 20) {
                    set_loader_target(LOADER_ACTIVE_RPM);
                    set_flywheel_target(FLYWHEEL_ACTIVE_TARGET_RPM);
                }
            } else if (shoot.shoot_state == ShootState::ANTIJAM) {
                set_loader_target(shoot.antijam_direction * LOADER_ACTIVE_RPM);
            } else {
                ASSERT(false, "Unknown shoot state.");
            }
            break;
        default:
            set_loader_target(0);
            set_flywheel_target(0);
            shoot.loader_delay_counter = 0;
    }
}

void ShootApp::calc_motor_outputs() {
    // TODO: Loader calculations.
    ramp_calc_output(flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp,
                     LOOP_PERIOD_MS * 0.001f);

    ramp_calc_output(flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
                     LOOP_PERIOD_MS * 0.001f);

    pid2_single_loop_control(
        flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid,
        flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp.output,
        flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
        LOOP_PERIOD_MS * 0.001f);

    pid2_single_loop_control(
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid,
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp.output,
        flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
        LOOP_PERIOD_MS * 0.001f);

    pid2_single_loop_control(loader_control.speed_pid,
                             shoot.loader_target_rpm *
                                 robot_config::gimbal_params::LOADER_GEAR_RATIO,
                             loader_control.feedback.rx_rpm,
                             LOOP_PERIOD_MS * 0.001f);
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

void ShootApp::set_shoot_mode(ShootActMode_t new_mode) {
    switch (new_mode) {
        case SHOOT_CEASE:
        case SHOOT_CONT:
            shoot.shoot_act_mode = new_mode;
            break;
        default:
            return;
    }
}

void ShootApp::set_loader_target(float new_target) {
    shoot.loader_target_rpm = new_target;
}

void ShootApp::set_flywheel_target(float new_target) {
    shoot.flywheel_target_rpm = new_target;
    ramp_set_target(flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp,
                    flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
                    -new_target);
    ramp_set_target(flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
                    flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
                    new_target);
}