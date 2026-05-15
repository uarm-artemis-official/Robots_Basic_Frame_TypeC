/*******************************************************************************
* @file           : Shoot_App.c
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include <cstring>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "pid.h"
#include "ramp.hpp"
#include "robot_config.hpp"
#include "shared_config.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

namespace {
    constexpr size_t LEFT_FLYWHEEL_INDEX = 0;
    constexpr size_t RIGHT_FLYWHEEL_INDEX = 1;
    constexpr size_t TOP_FLYWHEEL_INDEX = 2;
}  // namespace

namespace ShootApp {

    inline namespace v1 {

        ShootApp::ShootApp(
            MW_RTOS::IRTOS& _rtos, const apps::shoot::ShootConfig& config_ref,
            mc2::RobotMC& mc_ref,
            comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>&
                communication_ref,
            IAmmoLid& ammo_lid_ref, IMotors& motors_ref)
            : RTOSApp(_rtos),
              config(config_ref),
              mc(mc_ref),
              communication(communication_ref),
              ammo_lid(ammo_lid_ref),
              motors(motors_ref) {}

        void ShootApp::init() {
            ammo_lid.init();

            pid2_init(speed_loader_control.speed_pid,
                      config.loader_speed_pid_config);

            position_loader_control.is_processing_command = false;
            position_loader_control.current_relative_position = 0.0f;
            position_loader_control.at_target_counter = 0;
            pid2_init(position_loader_control.position_pid,
                      config.loader_position_pid_config);

            pid2_init(position_loader_control.speed_pid,
                      config.loader_speed_pid_config);

            for (size_t i = 0; i < 3; i++) {
                pid2_init(flywheel_controls[i].speed_pid,
                          config.flywheel_speed_pid_config);
                ramp_init(flywheel_controls[i].sp_ramp,
                          config.max_flywheel_accel.get());
                std::memset(&(flywheel_controls[i].feedback), 0,
                            sizeof(Motor_Feedback_t));
            }
            flywheel_controls[LEFT_FLYWHEEL_INDEX].stdid = SHOOT_LEFT_FRIC;
            flywheel_controls[RIGHT_FLYWHEEL_INDEX].stdid = SHOOT_RIGHT_FRIC;
            flywheel_controls[TOP_FLYWHEEL_INDEX].stdid = SHOOT_TOP_FRIC;

            std::memset(&(shoot), 0, sizeof(Shoot));
            shoot.shoot_state = ShootState::NORMAL;
            shoot.antijam_direction = 1;

            /* set shoot mode */
            set_shoot_mode(SHOOT_CEASE);
            ammo_lid.set_lid_status(ammo_lid::LidStatus::CLOSED);
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
            mc2::ShootCommand shoot_command;
            auto message_ts = mc.get_message(shoot_command);

            if (message_ts.has_value()) {
                ShootActMode_t shoot_mode =
                    static_cast<ShootActMode_t>(shoot_command.command_bits);
                set_shoot_mode(shoot_mode);

                uint8_t open_ammo_lid =
                    static_cast<uint8_t>(shoot_command.extra_bits);
                if (open_ammo_lid == 1) {
                    ammo_lid.set_lid_status(ammo_lid::LidStatus::OPEN);
                } else {
                    ammo_lid.set_lid_status(ammo_lid::LidStatus::CLOSED);
                }
            }
        }

        void ShootApp::get_motor_feedback() {
            mc2::MotorRead motor_read;
            auto message_ts = mc.peek_message(motor_read);
            if (message_ts.has_value()) {
                for (size_t j = 0; j < MAX_MOTOR_COUNT; j++) {
                    switch (motor_read.can_ids[j]) {
                        case SHOOT_LOADER:
                            motors.get_raw_feedback(SHOOT_LOADER,
                                                    motor_read.feedback[j],
                                                    &loader_feedback);
                            break;
                        case SHOOT_LEFT_FRIC:
                            motors.get_raw_feedback(
                                SHOOT_LEFT_FRIC, motor_read.feedback[j],
                                &flywheel_controls[LEFT_FLYWHEEL_INDEX]
                                     .feedback);
                            break;
                        case SHOOT_RIGHT_FRIC:
                            motors.get_raw_feedback(
                                SHOOT_RIGHT_FRIC, motor_read.feedback[j],
                                &flywheel_controls[RIGHT_FLYWHEEL_INDEX]
                                     .feedback);
                            break;
                        case SHOOT_TOP_FRIC:
                            motors.get_raw_feedback(
                                SHOOT_TOP_FRIC, motor_read.feedback[j],
                                &flywheel_controls[TOP_FLYWHEEL_INDEX]
                                     .feedback);
                            break;
                        default:
                            continue;
                    }
                }
            }
        }

        void ShootApp::detect_loader_stall() {
            int16_t current_loader_rpm = loader_feedback.rx_rpm;
            float current_loader_current = loader_feedback.rx_current;
            float current_loader_output = shoot.loader_output;

            if (fabs(current_loader_output) > 0 &&
                abs(current_loader_rpm) <
                    robot_config::shoot_params::JAM_LOADER_RPM_THRESHOLD &&
                fabs(current_loader_current - current_loader_output) /
                        current_loader_current <
                    robot_config::shoot_params::
                        JAM_LOADER_CURRENT_RELATIVE_DIFF_THRESHOLD) {
                shoot.stall_duration += ShootApp::get_loop_period();
            } else {
                shoot.stall_duration = 0;
                shoot.no_stall_duration += ShootApp::get_loop_period();
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

                    if (shoot.loader_delay_counter >= 100) {
                        set_flywheel_target(0);
                    }
                    shoot.loader_delay_counter =
                        value_limit(shoot.loader_delay_counter + 1, 0, 1000);
                    break;
                case SHOOT_CONT:
                    shoot.loader_delay_counter = 0;
                    if (shoot.shoot_state == ShootState::NORMAL) {
                        set_flywheel_target(config.active_flywheel_speed.get());
                        // TODO: Change so it waits for all flywheels to be at 80% speed.
                        float average_flywheel_rpm =
                            (fabs(flywheel_controls[LEFT_FLYWHEEL_INDEX]
                                      .feedback.rx_rpm) +
                             fabs(flywheel_controls[RIGHT_FLYWHEEL_INDEX]
                                      .feedback.rx_rpm) +
                             fabs(flywheel_controls[TOP_FLYWHEEL_INDEX]
                                      .feedback.rx_rpm)) /
                            3;
                        if (average_flywheel_rpm >=
                            config.active_flywheel_speed.get() * 0.8) {
                            // TODO: Change to accomodate hero loader (reverse direction).
                            set_loader_target(
                                -config.active_loader_speed.get());
                        }
                    } else if (shoot.shoot_state == ShootState::ANTIJAM) {
                        set_loader_target(shoot.antijam_direction *
                                          config.active_loader_speed.get());
                        set_flywheel_target(shoot.antijam_direction *
                                            config.active_flywheel_speed.get());
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
                             ShootApp::get_loop_period());

            ramp_calc_output(flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
                             ShootApp::get_loop_period());
            ramp_calc_output(flywheel_controls[TOP_FLYWHEEL_INDEX].sp_ramp,
                             ShootApp::get_loop_period());

            pid2_single_loop_control(
                flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid,
                flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp.output,
                flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
                ShootApp::get_loop_period());

            pid2_single_loop_control(
                flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid,
                flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp.output,
                flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
                ShootApp::get_loop_period());

            pid2_single_loop_control(
                flywheel_controls[TOP_FLYWHEEL_INDEX].speed_pid,
                flywheel_controls[TOP_FLYWHEEL_INDEX].sp_ramp.output,
                flywheel_controls[TOP_FLYWHEEL_INDEX].feedback.rx_rpm,
                ShootApp::get_loop_period());

            if constexpr (robot_config::shoot_params::
                              ENABLE_LOADER_POSITION_CONTROL) {
                int32_t encoder_position_delta = relative_difference(
                    position_loader_control.prev_encoder_position,
                    loader_feedback.rx_angle, 8192);
                position_loader_control.current_relative_position +=
                    static_cast<float>(encoder_position_delta) / 8192 /
                    robot_config::gimbal_params::LOADER_GEAR_RATIO;
                position_loader_control.prev_encoder_position =
                    loader_feedback.rx_angle;
                float position_pid_out = pid2_single_loop_control(
                    position_loader_control.position_pid, 0,
                    -position_loader_control.current_relative_position,
                    ShootApp::get_loop_period());
                shoot.loader_output = pid2_single_loop_control(
                    position_loader_control.speed_pid, position_pid_out,
                    rpm_to_radps(loader_feedback.rx_rpm),
                    ShootApp::get_loop_period());
            } else {
                if (shoot.loader_target_rpm == 0) {
                    speed_loader_control.speed_pid.i_out = 0;
                    speed_loader_control.speed_pid.prev_d_error = 0;
                    speed_loader_control.speed_pid.total_out = 0;
                    speed_loader_control.speed_pid.prev_total_out = 0;
                    shoot.loader_output = 0;
                } else {
                    shoot.loader_output = pid2_single_loop_control(
                        speed_loader_control.speed_pid,
                        shoot.loader_target_rpm *
                            robot_config::gimbal_params::LOADER_GEAR_RATIO,
                        loader_feedback.rx_rpm, ShootApp::get_loop_period());
                }
            }
        }

        void ShootApp::send_motor_outputs() {
            mc2::MotorSet motor_set {};
            motor_set.motor_can_volts[0] =
                speed_loader_control.speed_pid.total_out;
            motor_set.can_ids[0] = SHOOT_LOADER;
            motor_set.motor_can_volts[1] =
                flywheel_controls[LEFT_FLYWHEEL_INDEX].speed_pid.total_out;
            motor_set.can_ids[1] = SHOOT_LEFT_FRIC;
            motor_set.motor_can_volts[2] =
                flywheel_controls[RIGHT_FLYWHEEL_INDEX].speed_pid.total_out;
            motor_set.can_ids[2] = SHOOT_RIGHT_FRIC;
            motor_set.motor_can_volts[3] =
                flywheel_controls[TOP_FLYWHEEL_INDEX].speed_pid.total_out;
            motor_set.can_ids[3] = SHOOT_TOP_FRIC;
            mc.pub_message(motor_set);
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
            switch (config.flywheel_config) {
                case apps::shoot::FlywheelConfiguration::TRIPLE:
                    ramp_set_target(
                        flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
                        new_target);
                    ramp_set_target(
                        flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
                        new_target);
                    ramp_set_target(
                        flywheel_controls[TOP_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[TOP_FLYWHEEL_INDEX].feedback.rx_rpm,
                        new_target);
                    break;
                case apps::shoot::FlywheelConfiguration::DUAL:
                    ramp_set_target(
                        flywheel_controls[LEFT_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[LEFT_FLYWHEEL_INDEX].feedback.rx_rpm,
                        -new_target);
                    ramp_set_target(
                        flywheel_controls[RIGHT_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[RIGHT_FLYWHEEL_INDEX].feedback.rx_rpm,
                        new_target);
                    ramp_set_target(
                        flywheel_controls[TOP_FLYWHEEL_INDEX].sp_ramp,
                        flywheel_controls[TOP_FLYWHEEL_INDEX].feedback.rx_rpm,
                        0);
                    break;
                default:
                    ASSERT(false,
                           "Unknown shoot app config, using default flywheel "
                           "target "
                           "RPM.");
                    break;
            }
        }
    }  // namespace v1
}  // namespace ShootApp