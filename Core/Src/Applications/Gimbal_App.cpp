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
#include "apps_defines.h"
#include "motors.h"
#include "pid.h"
#include "ramp.hpp"
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

void GimbalApp::wait_for_motors() {
    MotorReadMessage_t read_message;
    while (message_center.peek_message(MOTOR_READ, &read_message, 0) == 0) {
        vTaskDelay(100);
    }

    get_motor_feedback();
    update_headings();
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

    ramp_init(motor_controls[GIMBAL_YAW_MOTOR_INDEX].sp_ramp,
              robot_config::gimbal_params::YAW_RAMP_MAX_CHANGE);

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

    ramp_init(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].sp_ramp,
              robot_config::gimbal_params::PITCH_RAMP_MAX_CHANGE);

    // Initialize non-zero Gimbal_t fields.
    memset(&gimbal, 0, sizeof(Gimbal_t));
    gimbal.yaw_ecd_center = robot_config::gimbal_params::YAW_ECD_CENTER;
    gimbal.pitch_ecd_center = robot_config::gimbal_params::PITCH_ECD_CENTER;

    init_folp_filter(&(gimbal.folp_f_yaw), 0.90f);
    init_folp_filter(&(gimbal.folp_f_pitch), 0.90f);

    gimbal.prev_gimbal_motor_mode = ENCODE_MODE;
    gimbal.gimbal_motor_mode = ENCODE_MODE;
    gimbal.gimbal_act_mode = INDPET_MODE;
    gimbal.gimbal_mode = IDLE_MODE;
}

bool GimbalApp::exit_calibrate_cond() {
    // TODO: Add speed condition (e.g. rpm has to be lower than 5)?
    return fabs(gimbal.yaw_rel_angle) <
           (robot_config::gimbal_params::EXIT_CALIBRATION_YAW_ANGLE_DELTA *
            DEGREE2RAD);
}

void GimbalApp::calibrate() {
    get_motor_feedback();
    update_ecd_angles();
    gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
    gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;

    float yaw_diff =
        calc_rel_angle(gimbal.yaw_target_angle, gimbal.yaw_rel_angle);
    float pitch_diff =
        calc_rel_angle(gimbal.pitch_target_angle, gimbal.pitch_rel_angle);

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
    get_rc_info();
    get_motor_feedback();

    if (is_imu_calibrated()) {
        get_imu_headings();
    } else {
        calc_imu_center();
    }

    get_commands();
    safe_mode_switch();

    update_headings();
    update_targets(gimbal_channels);

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
    gimbal.gimbal_mode = mode;
}
/*
 * @brief     determime the mode for gimbal actions:
 * 				follow gimbal (master) | follow chassis (slave) or independent
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: act mode
 * */
void GimbalApp::set_act_mode(BoardActMode_t mode) {
    gimbal.prev_gimbal_act_mode = gimbal.gimbal_act_mode;
    gimbal.gimbal_act_mode = mode;
}

/*
 * @brief 	  set motor mode: gyro | encoder
 * @param[in] gimbal: main gimbal handler
 * @param[in] mode: motor mode
 * */
void GimbalApp::set_motor_mode(GimbalMotorMode_t mode) {
    gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;
    gimbal.gimbal_motor_mode = mode;
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

void GimbalApp::set_gimbal_targets(float new_yaw_target_angle,
                                   float new_pitch_target_angle) {
    gimbal.yaw_target_angle = new_yaw_target_angle;
    gimbal.pitch_target_angle = new_pitch_target_angle;

    if (gimbal.yaw_target_angle > PI)
        gimbal.yaw_target_angle -= 2.0f * PI;
    if (gimbal.yaw_target_angle < -PI)
        gimbal.yaw_target_angle += 2.0f * PI;

    ramp_set_target(motor_controls[GIMBAL_YAW_MOTOR_INDEX].sp_ramp,
                    gimbal.yaw_rel_angle, gimbal.yaw_target_angle);
    ramp_set_target(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].sp_ramp,
                    gimbal.pitch_rel_angle, gimbal.pitch_target_angle);
}

void GimbalApp::get_rc_info() {
#ifndef ENABLE_MANUAL_MODE_SET
    RCInfoMessage_t rc_info;
    BaseType_t new_rc_info_message =
        message_center.peek_message(RC_INFO, &rc_info, 0);
    if (new_rc_info_message == pdTRUE) {
        set_modes(rc_info.modes);
        memcpy(gimbal_channels, rc_info.channels, sizeof(int16_t) * 2);
    }
#else
    set_board_mode(gbal, AUTO_AIM_MODE);
    set_act_mode(gbal, GIMBAL_FOLLOW);
    set_motor_mode(gbal, ENCODE_MODE);
#endif
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
    int16_t pitch_ecd_rel_angle = -GimbalApp::calc_ecd_rel_angle(
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_angle,
        gimbal.pitch_ecd_center);

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

void GimbalApp::calc_channels_to_angles(const int16_t g_channels[2],
                                        float deltas[2]) {
    /**
	 * deltas[0] - delta yaw
	 * deltas[1] - delta pitch
	 */
    deltas[0] = in_out_map((float) g_channels[0], -CHANNEL_OFFSET_MAX_ABS_VAL,
                           CHANNEL_OFFSET_MAX_ABS_VAL, -2.0f * DEGREE2RAD,
                           2.0f * DEGREE2RAD);
    deltas[1] = in_out_map((float) g_channels[1], -CHANNEL_OFFSET_MAX_ABS_VAL,
                           CHANNEL_OFFSET_MAX_ABS_VAL, -1.0f * DEGREE2RAD,
                           1.0f * DEGREE2RAD);
}

bool GimbalApp::is_imu_calibrated() {
    return gimbal.yaw_imu_center != 0.0f;
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

void GimbalApp::get_commands() {
    GimbalCommandMessage_t gimbal_command;
    uint8_t new_message =
        message_center.get_message(COMMAND_GIMBAL, &gimbal_command, 0);
    if (new_message == pdTRUE) {
        command_deltas[0] = gimbal_command.yaw;
        command_deltas[1] = gimbal_command.pitch;
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
    gimbal.yaw_imu_angle =
        GimbalApp::calc_rel_angle(gimbal.yaw_imu_center, gimbal.yaw_imu_angle);
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
        set_gimbal_targets(0, 0);
    } else if (gimbal.gimbal_mode == AUTO_AIM_MODE) {
        float deltas[2];
        BaseType_t new_pack_response =
            message_center.get_message(AUTO_AIM, deltas, 0);
        if (new_pack_response == pdTRUE) {
            set_gimbal_targets(gimbal.yaw_rel_angle + deltas[0],
                               gimbal.pitch_rel_angle + deltas[1]);
        }
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               gimbal.gimbal_act_mode == INDPET_MODE) {
        set_gimbal_targets(0, gimbal.pitch_target_angle);
    } else {
        // FIXME: Change subtracting command deltas to adding them.
        set_gimbal_targets(gimbal.yaw_target_angle - command_deltas[0],
                           gimbal.pitch_target_angle - command_deltas[1]);
    }

    // Software limit pitch target range to prevent hitting mechanical hard-stops.
    // gimbal.pitch_target_angle =
    //     value_limit(gimbal.pitch_target_angle, GimbalApp::PITCH_LOWER_LIMIT,
    //                 GimbalApp::PITCH_UPPER_LIMIT);
}

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void GimbalApp::cmd_exec() {
    ramp_calc_output(motor_controls[GIMBAL_YAW_MOTOR_INDEX].sp_ramp,
                     LOOP_PERIOD_MS * 0.001);
    ramp_calc_output(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].sp_ramp,
                     LOOP_PERIOD_MS * 0.001);

    float yaw_diff = GimbalApp::calc_rel_angle(
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].sp_ramp.output,
        gimbal.yaw_rel_angle);

    // Inverted relative angle calculation (i.e. find target relative to angle instead of reverse)
    // due to mounting of GM6020 on left side of gimbal instead of right side. The left side has
    // opposite rotation sign convention (i.e. rotating CCW is negative) than right side.
    float pitch_diff = GimbalApp::calc_rel_angle(
        gimbal.pitch_rel_angle,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].sp_ramp.output);

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