/*******************************************************************************
* @file           : Gimbal_App.c
* @brief          : gimbal task
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include <algorithm>
#include <cstddef>
#include <cstring>
#include <span>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "messages.hpp"
#include "pid.h"
#include "robot_config.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

GimbalApp::GimbalApp(
    MW_RTOS::IRTOS& _rtos, const apps::gimbal::GimbalConfig& config_ref,
    mc2::RobotMC& mc_ref,
    comm::Communication<mc2::RobotMC, mc2::RobotMC::Topics>& communication_ref,
    IEventCenter& event_center_ref, IMotors& motors_ref,
    modules::debug::Debug& _debug)
    : ExtendedRTOSApp(_rtos),
      config(config_ref),
      mc(mc_ref),
      communication(communication_ref),
      event_center(event_center_ref),
      motors(motors_ref),
      debug(_debug) {}

void GimbalApp::init() {
    /* init gimbal task */
    auto reserve_response = debug.reserve_debug_uart();
    if (reserve_response.has_value()) {
        debug_uart_access_token = reserve_response.value();
    }

    set_initial_state();

    set_board_mode(PATROL_MODE);
    set_act_mode(INDPET_MODE);
    set_motor_mode(ENCODE_MODE);

    wait_for_motors();
}

bool GimbalApp::calibrate_start_precondition() {
    // mc2::MotorRead motor_read;
    // auto message_ts = mc.peek_message(motor_read);
    // if (message_ts.has_value()) {
    //     bool yaw_motor = false;
    //     bool pitch_motor = false;
    //     for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
    //         yaw_motor = yaw_motor ||
    //                     (motor_read.can_ids[i] == Motor_CAN_ID_t::GIMBAL_YAW);
    //         pitch_motor = pitch_motor || (motor_read.can_ids[i] ==
    //                                       Motor_CAN_ID_t::GIMBAL_PITCH);
    //     }
    //     return yaw_motor && pitch_motor;
    // }
    // return false;
    return true;
}

void GimbalApp::wait_for_motors() {
    while (!calibrate_start_precondition()) {
        rtos.delay_ms(100);
    }
    rtos.delay_ms(200);
    get_motor_feedback();
    update_ecd_angles();
    gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
    gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;
}

void GimbalApp::set_initial_state() {
    memset(motor_controls, 0, sizeof(Gimbal_Motor_Control_t) * 2);
    motor_controls[GIMBAL_YAW_MOTOR_INDEX].stdid = GIMBAL_YAW;
    motor_controls[GIMBAL_PITCH_MOTOR_INDEX].stdid = GIMBAL_PITCH;
    pid2_init(motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid,
              config.yaw_position_pid_config);

    pid2_init(motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid,
              config.yaw_speed_pid_config);

    pid2_init(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid,
              config.pitch_position_pid_config);

    pid2_init(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid,
              config.pitch_speed_pid_config);

    // Initialize non-zero Gimbal_t fields.
    memset(&gimbal, 0, sizeof(Gimbal_t));
    gimbal.yaw_ecd_center = config.yaw_ecd_center;
    gimbal.pitch_ecd_center = config.pitch_ecd_center;

    init_folp_filter(&(gimbal.folp_f_yaw), config.imu_yaw_lpf_gain);
    init_folp_filter(&(gimbal.folp_f_pitch), config.imu_pitch_lpf_gain);
}

bool GimbalApp::exit_calibrate_cond() {
    // return fabs(gimbal.yaw_rel_angle) <
    //            degrees_to_radians(robot_config::gimbal_params::EXIT_CALIBRATION_YAW_ANGLE_DELTA) &&
    //        abs(motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm) < 2 &&
    //        fabs(gimbal.pitch_rel_angle) < degrees_to_radians(2.0f) &&
    //        abs(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm) < 2;
    return true;
}

void GimbalApp::calibrate() {
    get_motor_feedback();
    update_ecd_angles();

    gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
    gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;
    gimbal.yaw_target_angle = 0;
    gimbal.pitch_target_angle = 0;

    calc_control_signals();
    send_motor_volts();
}

void GimbalApp::loop() {
    std::byte debug_msg[] = {std::byte {0x1}, std::byte {0x2}, std::byte {0x3}};
    debug.send_debug_message(
        debug_uart_access_token,
        std::span<const std::byte>(debug_msg, sizeof(debug_msg)), 1);

    get_motor_feedback();
    get_chassis_movement();

    if (is_imu_calibrated()) {
        get_imu_headings();
    } else {
        calc_imu_center();
    }

    safe_mode_switch();
    process_commands();

    update_headings();
    update_targets();

    // cmd_exec();
    calc_control_signals();

    send_motor_volts();
    send_rel_angles();
}

/*
 * @brief     set the gimbal board work mode:
 * 				patrol | detected armor | Auto_Poilt | IDLE(no action) | Debug(remote control)
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: Board work mode
 * */
void GimbalApp::set_board_mode(BoardMode_t mode) {
    switch (mode) {
        case PATROL_MODE:
        case AUTO_AIM_MODE:
        case AUTO_PILOT_MODE:  // full control to mini-pc.
        case IDLE_MODE:
            gimbal.gimbal_mode = mode;
            break;
        default:
            return;
    }
}
/*
 * @brief     determime the mode for gimbal actions:
 * 				follow gimbal (master) | follow chassis (slave) or independent
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: act mode
 * */
void GimbalApp::set_act_mode(BoardActMode_t mode) {
    switch (mode) {
        case GIMBAL_CENTER:
        case GIMBAL_FOLLOW:
        case SELF_GYRO:
        case INDPET_MODE:
            gimbal.prev_gimbal_act_mode = gimbal.gimbal_act_mode;
            gimbal.gimbal_act_mode = mode;
            break;
        default:
            return;
    }
}

/*
 * @brief 	  set motor mode: gyro | encoder
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: motor mode
 * */
void GimbalApp::set_motor_mode(GimbalMotorMode_t mode) {
    switch (mode) {
        case GYRO_MODE:
        case ENCODE_MODE:
            gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;
            gimbal.gimbal_motor_mode = mode;
            break;
        default:
            return;
    }
}

/*
 * @brief     mode selection based on remote controller
 * @param[in] chassis: main chassis handler
 * @param[in] rc: main remote controller handler
 * */
void GimbalApp::set_modes(uint8_t modes[3]) {
    BoardMode_t board_mode = static_cast<BoardMode_t>(modes[0]);
    BoardActMode_t act_mode = static_cast<BoardActMode_t>(modes[1]);

    set_board_mode(board_mode);
    if (is_imu_calibrated()) {
        set_act_mode(act_mode);
    } else {
        set_act_mode(INDPET_MODE);
    }

    gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;
    if (gimbal.gimbal_act_mode == SELF_GYRO ||
        gimbal.gimbal_act_mode == GIMBAL_FOLLOW ||
        gimbal.gimbal_act_mode == GIMBAL_CENTER) {
        set_motor_mode(GYRO_MODE);
    } else {
        set_motor_mode(ENCODE_MODE);
    }
}

void GimbalApp::get_motor_feedback() {
    mc2::MotorRead motor_read;
    Motor_CAN_ID_t gimbal_can_ids[] = {GIMBAL_YAW, GIMBAL_PITCH};

    auto message_ts = mc.peek_message(motor_read);
    if (message_ts.has_value()) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (gimbal_can_ids[i] == motor_read.can_ids[j]) {
                    motors.get_raw_feedback(gimbal_can_ids[i],
                                            motor_read.feedback[j],
                                            &(motor_controls[i].feedback));
                    break;
                }
            }
            // ASSERT(good == 0,
            //        "Gimbal motor ID is not provided in MOTOR_READ topic.");
        }
    }
}

void GimbalApp::get_chassis_movement() {
    mc2::ChassisMovement chassis_movement_msg;
    auto message_ts = mc.get_message(chassis_movement_msg);
    if (message_ts.has_value()) {
        // wz (angular velocity about z-axis) is positive for counter-clockwise rotation
        // and negative for clockwise rotation when viewed from above.
        // This is the only component currently used by the gimbal system.
        chassis_wz = chassis_movement_msg.wz;
    }
}

/*
 * @brief     Copy the gyroscope data from imu and calculate quaternion
 * 			  and euler's absolute angle through attitude-breakdown algorithms.
 * @param[in] gbal: main gimbal handler
 * */
void GimbalApp::update_imu_angle(float yaw, float pitch) {
    gimbal.yaw_imu_angle =
        first_order_low_pass_filter(&(gimbal.folp_f_yaw), yaw);
    gimbal.pitch_imu_angle =
        first_order_low_pass_filter(&(gimbal.folp_f_pitch), pitch);
}

/**
 * Calculates the smallest relative angle of angle2 relative to angle1.
 *
 * Assumes angle1 and angle2 are both in radians ([0, 2 * pi] or (-pi, pi).
 * angle1 and angle2 must have the same origin!!!
 *
 * The resulting angle assumes clockwise is negative and counter-clockwise is positive.
 */
float GimbalApp::calc_rel_angle(float angle1, float angle2) {
    float cw_magnitude = angle1 - angle2;
    if (cw_magnitude < 0)
        cw_magnitude += 2 * pi;

    float ccw_magnitude = angle2 - angle1;
    if (ccw_magnitude < 0)
        ccw_magnitude += 2 * pi;

    if (cw_magnitude < ccw_magnitude) {
        return -cw_magnitude;
    } else {
        return ccw_magnitude;
    }
}

/*
 * @brief     Get relative angle of gimbal motors.
 * @param[in] raw_ecd: abs yaw ecd angle from feedback
 * @param[in] center_offset: the center offset of ecd mode
 * */
int16_t GimbalApp::calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset) {
    /* declare a 16-bit signed integer tmp to store the relative angle */
    int16_t tmp = 0;

    /*  check if the center offset is in the upper half of the ecd range (4096-8191) */
    if (center_offset >= 4096) {
        /*  check if the raw ecd value is in the same half circle as the center offset */
        if (raw_ecd > center_offset - 4096)
            /*  the raw ecd value is in the same half circle as the center offset
          so, simply subtract the center offset from the raw ecd to get the relative angle */
            tmp = raw_ecd - center_offset;
        else
            /*  the raw ecd value is in the different half circle from the center offset
          subtract the center offset from the raw ecd plus 8192 to get the relative angle */
            tmp = raw_ecd + 8192 - center_offset;
    }
    /*  check if the center offset is in the lower half of the ecd range (0-4095) */
    else {
        /*  check if the raw ecd value is in the different half circle from the center offset */
        if (raw_ecd > center_offset + 4096)
            /*  the raw ecd value is in the different half circle from the center offset
          subtract the center offset and 8192 from the raw ecd to get the relative angle */
            tmp = raw_ecd - 8192 - center_offset;
        else
            /*  the raw ecd value is in the same half circle as the center offset
          so, simply subtract the center offset from the raw ecd to get the relative angle */
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}
/*
 * @brief     Update gimbal motor relative and mapped angle using encoder
 * @param[in] gbal: main gimbal handler
 * */
void GimbalApp::update_ecd_angles() {
    int16_t yaw_ecd_rel_angle = GimbalApp::calc_ecd_rel_angle(
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_angle,
        gimbal.yaw_ecd_center);

    // Depending on the motor orientation on the robot, we may need to invert
    int16_t pitch_ecd_rel_angle =
        config.upwards_pitch_orientation *
        GimbalApp::calc_ecd_rel_angle(
            motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_angle,
            gimbal.pitch_ecd_center);

    gimbal.yaw_ecd_angle = in_out_map(yaw_ecd_rel_angle, -4095, 4096, -pi, pi);
    gimbal.pitch_ecd_angle =
        in_out_map(pitch_ecd_rel_angle, -4095, 4096, -pi, pi);
}

/*
 * @brief     Ensure the mode switch safely
 * @param[in] gbal: main gimbal handler
 * */
void GimbalApp::safe_mode_switch() {
    if (gimbal.prev_gimbal_motor_mode != gimbal.gimbal_motor_mode) {
        // TODO: Implement synchronization method if switching from encoder -> gyro results
        //       in noticeable shifts due to changes in imu_center due to gyroscopic drift.
    }
}

bool GimbalApp::is_imu_calibrated() {
    return fabs(gimbal.yaw_imu_center) > 0.0001;
}

void GimbalApp::calc_imu_center() {
    if (gimbal.yaw_imu_center_sample_count ==
        GimbalApp::IMU_CENTER_TARGET_SAMPLES) {
        gimbal.yaw_imu_center =
            gimbal.yaw_imu_center_cumsum / gimbal.yaw_imu_center_sample_count;
        return;
    }

    mc2::ImuReadings imu_readings;
    auto message_ts = mc.get_message(imu_readings);
    if (message_ts.has_value()) {
        gimbal.yaw_imu_center_cumsum += imu_readings.yaw;
        gimbal.yaw_imu_center_sample_count++;
    }
}

void GimbalApp::get_imu_headings() {
    mc2::ImuReadings imu_readings;
    auto message_ts = mc.get_message(imu_readings);
    if (message_ts.has_value()) {
        update_imu_angle(imu_readings.yaw, imu_readings.pitch);
    } else {
        // TODO: Implement error handling
    }
}

void GimbalApp::process_commands() {
    mc2::GimbalCommand gimbal_command;
    auto message_ts = mc.get_message(gimbal_command);
    if (message_ts.has_value()) {
        // TODO: Convert deltas to use strong types.
        command_deltas[0] = gimbal_command.delta_yaw.get();
        command_deltas[1] = gimbal_command.delta_pitch.get();

        if (fabs(command_deltas[0]) < 0.001)
            command_deltas[0] = 0;
        if (fabs(command_deltas[1]) < 0.001)
            command_deltas[1] = 0;

        BoardMode_t board_mode =
            static_cast<BoardMode_t>((gimbal_command.command_bits >> 3) & 0x7);
        BoardActMode_t act_mode =
            static_cast<BoardActMode_t>(gimbal_command.command_bits & 0x7);

        set_board_mode(board_mode);
        set_act_mode(act_mode);

        gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;
        if (gimbal.gimbal_act_mode == SELF_GYRO ||
            gimbal.gimbal_act_mode == GIMBAL_FOLLOW ||
            gimbal.gimbal_act_mode == GIMBAL_CENTER) {
            set_motor_mode(GYRO_MODE);
        } else {
            set_motor_mode(ENCODE_MODE);
        }
    } else {
        command_deltas[0] = 0;
        command_deltas[1] = 0;
    }
}

void GimbalApp::send_rel_angles() {
    mc2::GimbalRelativeAngles relative_angles(gimbal.yaw_ecd_angle,
                                              gimbal.pitch_ecd_angle);
    communication.transmit_external_message(relative_angles,
                                            simple_comm::NodeID::Gimbal,
                                            simple_comm::NodeID::Chassis);
}

void GimbalApp::update_headings() {
    if (is_imu_calibrated()) {
        gimbal.yaw_imu_angle = GimbalApp::calc_rel_angle(gimbal.yaw_imu_center,
                                                         gimbal.yaw_imu_angle);
    }
    update_ecd_angles();

    if (gimbal.gimbal_motor_mode == GYRO_MODE) {
        gimbal.yaw_rel_angle = gimbal.yaw_imu_angle;
        gimbal.pitch_rel_angle = gimbal.pitch_imu_angle;
    } else if (gimbal.gimbal_motor_mode == ENCODE_MODE) {
        gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
        gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;
    } else {
        // TODO: Implement error handling / undefined motor mode state.
    }
}

void GimbalApp::update_targets() {
    if (gimbal.gimbal_mode == IDLE_MODE) {
        gimbal.yaw_target_angle = 0;
        gimbal.pitch_target_angle = 0;
    } else if (gimbal.gimbal_mode == AUTO_AIM_MODE) {
        mc2::AutoAim auto_aim;
        auto message_ts = mc.get_message(auto_aim);
        if (message_ts.has_value()) {
            gimbal.yaw_target_angle = gimbal.yaw_rel_angle + auto_aim.delta_yaw;
            gimbal.pitch_target_angle =
                gimbal.pitch_rel_angle + auto_aim.delta_pitch;

            if (gimbal.yaw_target_angle > pi)
                gimbal.yaw_target_angle -= 2.0f * pi;
            if (gimbal.yaw_target_angle < -pi)
                gimbal.yaw_target_angle += 2.0f * pi;
        }
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               gimbal.gimbal_act_mode == INDPET_MODE) {
        gimbal.yaw_target_angle = 0;
        gimbal.pitch_target_angle =
            value_limit(gimbal.pitch_target_angle + command_deltas[1],
                        config.min_pitch_angle,
                        config.max_pitch_angle);  //- command_deltas[1]
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               (gimbal.gimbal_act_mode == GIMBAL_FOLLOW ||
                gimbal.gimbal_act_mode == GIMBAL_CENTER ||
                gimbal.gimbal_act_mode == SELF_GYRO)) {
        gimbal.yaw_target_angle -= command_deltas[0];
        if (gimbal.yaw_target_angle > pi)
            gimbal.yaw_target_angle -= 2.0f * pi;
        if (gimbal.yaw_target_angle < -pi)
            gimbal.yaw_target_angle += 2.0f * pi;

        gimbal.pitch_target_angle =
            value_limit(gimbal.pitch_target_angle + command_deltas[1],
                        config.min_pitch_angle,
                        config.max_pitch_angle);  //- command_deltas[1]
        if (fabs(command_deltas[1]) > 0.001) {
            // limit_pitch_target();
        }
    } else {
        gimbal.yaw_target_angle = 0;
        gimbal.pitch_target_angle = 0;
        // ASSERT(false, "Unknown state");
    }
}

// TODO: Small shaking/doesn't fully limit pitch within range.
void GimbalApp::limit_pitch_target() {
    gimbal.pitch_target_angle =
        value_limit(gimbal.pitch_target_angle,
                    robot_config::gimbal_config.min_pitch_angle -
                        (gimbal.pitch_rel_angle - gimbal.pitch_ecd_angle),
                    robot_config::gimbal_config.max_pitch_angle -
                        (gimbal.pitch_rel_angle - gimbal.pitch_ecd_angle));
}

void GimbalApp::calc_control_signals() {
    // All calculations are done with right hand rule gimbal orientation:
    //  - Yaw: CCW positive (i.e. CCW rotation yaws left)
    //  - Pitch: CCW positive (i.e. CCW rotation pitches up).
    float yaw_diff = GimbalApp::calc_rel_angle(gimbal.yaw_target_angle,
                                               gimbal.yaw_rel_angle);
    float pitch_diff;
    pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_target_angle,
                                           gimbal.pitch_rel_angle);
    pid2_dual_loop_control(
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid, 0, yaw_diff,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm -
            chassis_wz.get(),
        GimbalApp::get_loop_period(), GimbalApp::get_loop_period());

    pid2_dual_loop_control(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid, 0, pitch_diff,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm,
        GimbalApp::get_loop_period(), GimbalApp::get_loop_period());
}

void GimbalApp::send_motor_volts() {
    mc2::MotorSet motor_set;
    // Yaw
    motor_set.motor_can_volts[0] = (int32_t) motor_controls[0].s_pid.total_out;
    motor_set.can_ids[0] = (Motor_CAN_ID_t) motor_controls[0].stdid;

    // Pitch
    motor_set.motor_can_volts[1] =
        (int32_t) (motor_controls[1].s_pid.total_out *
                   config.upwards_pitch_orientation);
    motor_set.can_ids[1] = (Motor_CAN_ID_t) motor_controls[1].stdid;

    mc.pub_message(motor_set);
}
