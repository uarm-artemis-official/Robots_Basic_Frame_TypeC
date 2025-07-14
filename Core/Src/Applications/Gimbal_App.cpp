/*******************************************************************************
* @file           : Gimbal_App.c
* @brief          : gimbal task
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#include "Gimbal_App.h"
#include <string.h>
#include <algorithm>
#include "apps_defines.h"
#include "pid.h"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

GimbalApp::GimbalApp(IMessageCenter& message_center_ref,
                     IEventCenter& event_center_ref, IDebug& debug_ref,
                     IMotors& motors_ref)
    : message_center(message_center_ref),
      event_center(event_center_ref),
      debug(debug_ref),
      motors(motors_ref) {}

void GimbalApp::init() {
    debug.set_led_state(BLUE, ON);
    /* init gimbal task */
    set_initial_state();

    set_board_mode(PATROL_MODE);
    set_act_mode(INDPET_MODE);
    set_motor_mode(ENCODE_MODE);

    wait_for_motors();
}

bool GimbalApp::calibrate_start_precondition() {
    MotorReadMessage_t read_message;
    uint8_t new_motor_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_motor_message == pdTRUE) {
        bool yaw_motor = false;
        bool pitch_motor = false;
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
            yaw_motor = yaw_motor ||
                        (read_message.can_ids[i] == Motor_CAN_ID_t::GIMBAL_YAW);
            pitch_motor = pitch_motor || (read_message.can_ids[i] ==
                                          Motor_CAN_ID_t::GIMBAL_PITCH);
        }
        return yaw_motor && pitch_motor;
    }
    return false;
}

void GimbalApp::wait_for_motors() {
    while (!calibrate_start_precondition()) {
        vTaskDelay(100);
    }
    vTaskDelay(200);
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
              robot_config::gimbal_params::KP_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::KI_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::KD_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::BETA_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::YETA_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::MIN_OUT_GIMBAL_YAW_ANGLE,
              robot_config::gimbal_params::MAX_OUT_GIMBAL_YAW_ANGLE);

    pid2_init(motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid,
              robot_config::gimbal_params::KP_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::KI_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::KD_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::BETA_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::YETA_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::MIN_OUT_GIMBAL_YAW_SPEED,
              robot_config::gimbal_params::MAX_OUT_GIMBAL_YAW_SPEED);

    pid2_init(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid,
              robot_config::gimbal_params::KP_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::KI_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::KD_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::BETA_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::YETA_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::MIN_OUT_GIMBAL_PITCH_ANGLE,
              robot_config::gimbal_params::MAX_OUT_GIMBAL_PITCH_ANGLE);

    pid2_init(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid,
              robot_config::gimbal_params::KP_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::KI_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::KD_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::BETA_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::YETA_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::MIN_OUT_GIMBAL_PITCH_SPEED,
              robot_config::gimbal_params::MAX_OUT_GIMBAL_PITCH_SPEED);

    // Initialize non-zero Gimbal_t fields.
    memset(&gimbal, 0, sizeof(Gimbal_t));
    gimbal.yaw_ecd_center = robot_config::gimbal_params::YAW_ECD_CENTER;
    gimbal.pitch_ecd_center = robot_config::gimbal_params::PITCH_ECD_CENTER;

    // TODO: Move 'a' constants to robot_config.
    init_folp_filter(&(gimbal.folp_f_yaw), 0.90f);
    init_folp_filter(&(gimbal.folp_f_pitch), 1.0f);
}

bool GimbalApp::exit_calibrate_cond() {
    return fabs(gimbal.yaw_rel_angle) <
               (robot_config::gimbal_params::EXIT_CALIBRATION_YAW_ANGLE_DELTA *
                DEGREE2RAD) &&
           abs(motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm) < 2 &&
           fabs(gimbal.pitch_rel_angle) < (2.0f * DEGREE2RAD) &&
           abs(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm) < 2;
}

void GimbalApp::calibrate() {
    get_motor_feedback();
    update_ecd_angles();
    gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
    gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;

    float yaw_diff =
        calc_rel_angle(gimbal.yaw_target_angle, gimbal.yaw_rel_angle);

    // TODO: Find a better way...
#if !defined(HERO_GIMBAL)
    float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_rel_angle,
                                                 gimbal.pitch_target_angle);
#else
    float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_target_angle,
                                                 gimbal.pitch_rel_angle);
#endif

    pid2_dual_loop_control(
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid, 0, yaw_diff,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    pid2_dual_loop_control(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid, 0, pitch_diff,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    send_motor_volts();
}

void GimbalApp::loop() {
    get_motor_feedback();

    if (is_imu_calibrated()) {
        get_imu_headings();
    } else {
        calc_imu_center();
    }

    safe_mode_switch();
    process_commands();

    update_headings();
    update_targets();

    cmd_exec();

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
    MotorReadMessage_t read_message;
    Motor_CAN_ID_t gimbal_can_ids[] = {GIMBAL_YAW, GIMBAL_PITCH};

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (gimbal_can_ids[i] == read_message.can_ids[j]) {
                    motors.get_raw_feedback(gimbal_can_ids[i],
                                            read_message.feedback[j],
                                            &(motor_controls[i].feedback));
                    break;
                }
            }
            // ASSERT(good == 0,
            //        "Gimbal motor ID is not provided in MOTOR_READ topic.");
        }
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
 * Assumes angle1 and angle2 are both in radians ([0, 2 * PI] or (-PI, PI).
 * angle1 and angle2 must have the same origin!!!
 *
 * The resulting angle assumes clockwise is negative and counter-clockwise is positive.
 */
float GimbalApp::calc_rel_angle(float angle1, float angle2) {
    float cw_magnitude = angle1 - angle2;
    if (cw_magnitude < 0)
        cw_magnitude += 2 * PI;

    float ccw_magnitude = angle2 - angle1;
    if (ccw_magnitude < 0)
        ccw_magnitude += 2 * PI;

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

// Negated pitch angle due to mounting of GM6020 on left side.
// Left-side mounting flips rotation sign convention (i.e. CCW is negative).
// Negating pitch restores regular sign convention.

// TODO: Find better way...
#if !defined(HERO_GIMBAL)
    int16_t pitch_ecd_rel_angle = -GimbalApp::calc_ecd_rel_angle(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_angle,
        gimbal.pitch_ecd_center);
#else
    int16_t pitch_ecd_rel_angle = GimbalApp::calc_ecd_rel_angle(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_angle,
        gimbal.pitch_ecd_center);
#endif

    gimbal.yaw_ecd_angle = in_out_map(yaw_ecd_rel_angle, -4095, 4096, -PI, PI);
    gimbal.pitch_ecd_angle =
        in_out_map(pitch_ecd_rel_angle, -4095, 4096, -PI, PI);
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

    float attitude[2];
    BaseType_t new_imu_message =
        message_center.get_message(IMU_READINGS, attitude, 0);
    if (new_imu_message) {
        gimbal.yaw_imu_center_cumsum += attitude[0];
        gimbal.yaw_imu_center_sample_count++;
    }
}

void GimbalApp::get_imu_headings() {
    float imu_readings[2];
    BaseType_t new_imu_message =
        message_center.peek_message(IMU_READINGS, imu_readings, 0);

    if (new_imu_message == pdTRUE) {
        update_imu_angle(imu_readings[0], imu_readings[1]);
    } else {
        // TODO: Implement error handling
    }
}

void GimbalApp::process_commands() {
    GimbalCommandMessage_t gimbal_command;
    uint8_t new_message =
        message_center.get_message(COMMAND_GIMBAL, &gimbal_command, 0);
    if (new_message == pdTRUE) {
        command_deltas[0] = gimbal_command.yaw;
        command_deltas[1] = gimbal_command.pitch;

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
    CANCommMessage_t rel_angle_message;
    rel_angle_message.topic_name = GIMBAL_REL_ANGLES;

    memcpy(rel_angle_message.data, &gimbal.yaw_ecd_angle, sizeof(float));
    memcpy(&(rel_angle_message.data[4]), &(gimbal.pitch_rel_angle),
           sizeof(float));

    message_center.pub_message(COMM_OUT, &rel_angle_message);
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
        float deltas[2];
        BaseType_t new_pack_response =
            message_center.get_message(AUTO_AIM, deltas, 0);
        if (new_pack_response == pdTRUE) {
            gimbal.yaw_target_angle = gimbal.yaw_rel_angle + deltas[0];
            gimbal.pitch_target_angle = gimbal.pitch_rel_angle + deltas[1];

            if (gimbal.yaw_target_angle > PI)
                gimbal.yaw_target_angle -= 2.0f * PI;
            if (gimbal.yaw_target_angle < -PI)
                gimbal.yaw_target_angle += 2.0f * PI;
        }
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               gimbal.gimbal_act_mode == INDPET_MODE) {
        gimbal.yaw_target_angle = 0;
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               (gimbal.gimbal_act_mode == GIMBAL_FOLLOW ||
                gimbal.gimbal_act_mode == GIMBAL_CENTER ||
                gimbal.gimbal_act_mode == SELF_GYRO)) {
        gimbal.yaw_target_angle -= command_deltas[0];
        if (gimbal.yaw_target_angle > PI)
            gimbal.yaw_target_angle -= 2.0f * PI;
        if (gimbal.yaw_target_angle < -PI)
            gimbal.yaw_target_angle += 2.0f * PI;

        gimbal.pitch_target_angle = value_limit(
            gimbal.pitch_target_angle - command_deltas[1], -PI / 2, PI / 2);
        if (fabs(command_deltas[1]) > 0.001) {
            limit_pitch_target();
        }
    } else {
        gimbal.yaw_target_angle = 0;
        gimbal.pitch_target_angle = 0;
        // ASSERT(false, "Unknown state");
    }
}

// TODO: Small shaking/doesn't fully limit pitch within range.
void GimbalApp::limit_pitch_target() {
    static_assert(-PI / 2 <= robot_config::gimbal_params::PITCH_MIN_ANGLE);
    static_assert(robot_config::gimbal_params::PITCH_MAX_ANGLE <= PI / 2);
    gimbal.pitch_target_angle =
        value_limit(gimbal.pitch_target_angle,
                    robot_config::gimbal_params::PITCH_MIN_ANGLE -
                        (gimbal.pitch_rel_angle - gimbal.pitch_ecd_angle),
                    robot_config::gimbal_params::PITCH_MAX_ANGLE -
                        (gimbal.pitch_rel_angle - gimbal.pitch_ecd_angle));
}

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void GimbalApp::cmd_exec() {
    float yaw_diff = GimbalApp::calc_rel_angle(gimbal.yaw_target_angle,
                                               gimbal.yaw_rel_angle);

// Inverted relative angle calculation (i.e. find target relative to angle instead of reverse)
// due to mounting of GM6020 on left side of gimbal instead of right side. The left side has
// opposite rotation sign convention (i.e. rotating CCW is negative) than right side.
// float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_target_angle,
//                                              gimbal.pitch_rel_angle);

// TODO: Find better way...
#if !defined(HERO_GIMBAL)
    float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_rel_angle,
                                                 gimbal.pitch_target_angle);
#else
    float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_target_angle,
                                                 gimbal.pitch_rel_angle);
#endif

    pid2_dual_loop_control(
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid, 0, yaw_diff,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    pid2_dual_loop_control(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid, 0, pitch_diff,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);
}

void GimbalApp::send_motor_volts() {
    MotorSetMessage_t set_message;
    memset(&set_message, 0, sizeof(MotorSetMessage_t));

    for (int i = 0; i < 2; i++) {
        set_message.motor_can_volts[i] =
            (int32_t) motor_controls[i].s_pid.total_out;
        set_message.can_ids[i] = (Motor_CAN_ID_t) motor_controls[i].stdid;
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}