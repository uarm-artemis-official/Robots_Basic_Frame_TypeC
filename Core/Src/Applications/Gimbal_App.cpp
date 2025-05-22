/*******************************************************************************
* @file           : Gimbal_App.c
* @brief          : gimbal task
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __GIMBAL_APP_C__
#define __GIMBAL_APP_C__

#include "Gimbal_App.h"
#include <string.h>
#include "apps_defines.h"
#include "debug.h"
#include "event_center.h"
#include "message_center.h"
#include "motors.h"
#include "pid.h"
#include "public_defines.h"
#include "ramp.h"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

GimbalApp::GimbalApp(IMessageCenter& message_center_ref,
                     IEventCenter& event_center_ref, IDebug& debug_ref)
    : message_center(message_center_ref),
      event_center(event_center_ref),
      debug(debug_ref) {}

void GimbalApp::init() {
    debug.set_led_state(BLUE, ON);
    /* init gimbal task */
    set_initial_state();

    set_board_mode(PATROL_MODE);
    set_act_mode(INDPET_MODE);
    set_motor_mode(ENCODE_MODE);
}

bool GimbalApp::exit_calibrate_cond() {
    return fabs(gimbal.yaw_rel_angle) < (2.0f * DEGREE2RAD);
}

void GimbalApp::calibrate() {
    get_motor_feedback();
    update_headings();

    //		float yaw_diff = gimbal_calc_ecd_rel_angle(gbal->yaw_ecd_center, g_motors[GIMBAL_YAW_MOTOR_INDEX].motor_feedback.rx_angle);
    //		float pitch_diff = gimbal_calc_ecd_rel_angle(gbal->pitch_ecd_center, g_motors[GIMBAL_PITCH_MOTOR_INDEX].motor_feedback.rx_angle);
    float yaw_diff =
        calc_rel_angle(gimbal.yaw_target_angle, gimbal.yaw_rel_angle);
    float pitch_diff =
        calc_rel_angle(gimbal.pitch_target_angle, gimbal.pitch_rel_angle);

    pid2_dual_loop_control(
        &(motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid),
        &(motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid), 0, yaw_diff,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    pid2_dual_loop_control(
        &(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid),
        &(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid), 0, pitch_diff,
        motor_controls[GIMBAL_PITCH_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    send_motor_volts();
}

void GimbalApp::after_calibrate() {
    EventBits_t res = 0;
    while ((res & IMU_READY) != IMU_READY) {
        res = event_center.wait_events(IMU_READY, 100000);
    }
}

bool GimbalApp::exit_loop_prepare_cond() {
    return imu_calibration.sample_count >= GIMBAL_IMU_SAMPLES;
}

void GimbalApp::loop_prepare() {
    float headings[2];
    BaseType_t imu_posting =
        message_center.peek_message(IMU_READINGS, headings, 0);
    if (imu_posting == pdTRUE) {
        update_imu_angle(headings[0], headings[1]);
        imu_calibration.yaw_samples_cumsum += gimbal.yaw_imu_angle;
        imu_calibration.pitch_samples_cumsum += gimbal.pitch_imu_angle;
        imu_calibration.sample_count++;
    }
}

void GimbalApp::after_loop_prepare() {
    gimbal.yaw_imu_center =
        imu_calibration.yaw_samples_cumsum / imu_calibration.sample_count;
    //  TODO: Uncomment after implementing pitch centering.
    // gimbal.pitch_imu_center =
    //     imu_calibration.pitch_samples_cumsum / imu_calibration.sample_count;
}

/**
 * Assumptions before entering loop:
 *  - IMU is active and posting attitude to IMU_READINGS topic.
 *  - Gimbal is calibrated and centered at relative 0.
 *  - Gimbal IMU centers for yaw and pitch are set.
 */
void GimbalApp::loop() {
    get_rc_info();
    get_motor_feedback();
    get_imu_headings();

    safe_mode_switch();
    update_headings();
    update_targets(gimbal_channels);

    /* Update previous mode */
    gimbal.prev_gimbal_motor_mode = gimbal.gimbal_motor_mode;

    cmd_exec();

    send_motor_volts();
    send_rel_angles();
}

void GimbalApp::set_initial_state() {
    memset(motor_controls, 0, sizeof(Gimbal_Motor_Control_t) * 2);
    motor_controls[GIMBAL_YAW_MOTOR_INDEX].stdid = GIMBAL_YAW;
    motor_controls[GIMBAL_PITCH_MOTOR_INDEX].stdid = GIMBAL_PITCH;
    pid2_init(&(motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid), kp_angle_yaw,
              ki_angle_yaw, kd_angle_yaw, beta_angle_yaw, yeta_angle_yaw,
              -max_out_angle_yaw, max_out_angle_yaw);
    pid2_init(&(motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid), kp_spd_yaw,
              ki_spd_yaw, kd_spd_yaw, beta_spd_yaw, yeta_spd_yaw,
              -max_out_spd_yaw, max_out_spd_yaw);
    pid2_init(&(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid), kp_angle_pitch,
              ki_angle_pitch, kd_angle_pitch, beta_angle_pitch,
              yeta_angle_pitch, -max_out_angle_pitch, max_out_angle_pitch);
    pid2_init(&(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid), kp_spd_pitch,
              ki_spd_pitch, kd_spd_pitch, beta_spd_pitch, yeta_spd_pitch,
              -max_out_spd_pitch, max_out_spd_pitch);

    // Initialize non-zero Gimbal_t fields.
    memset(&gimbal, 0, sizeof(Gimbal_t));
    gimbal.yaw_ecd_center =
        YAW_ECD_CENTER;  // center position of the yaw motor - encoder
    gimbal.pitch_ecd_center = PITCH_ECD_CENTER;

    init_folp_filter(&(gimbal.folp_f_yaw), 0.90f);
    init_folp_filter(&(gimbal.folp_f_pitch), 0.99f);

    init_ewma_filter(&(gimbal.ewma_f_x), 0.8f);         //0.65 for older client
    init_ewma_filter(&(gimbal.ewma_f_y), 0.8f);         //0.6 for older client
    init_ewma_filter(&(gimbal.ewma_f_aim_yaw), 0.95f);  //0.65 for older client
    init_ewma_filter(&(gimbal.ewma_f_aim_pitch), 0.95f);  //0.6 for older client

    //	init_swm_filter(&gbal->swm_f_x, 20);// window size 50
    //	init_swm_filter(&gbal->swm_f_y, 20);

    memset(&(gimbal.ahrs_sensor), 0, sizeof(AhrsSensor_t));
    memset(&(gimbal.euler_angle), 0, sizeof(Attitude_t));

    gimbal.prev_gimbal_motor_mode = ENCODE_MODE;

    ramp_init(&(gimbal.yaw_ramp), 1500);  //1.5s init
    ramp_init(&(gimbal.pitch_ramp), 1500);

    // Center gimbal
    gimbal.yaw_target_angle = 0;
    gimbal.pitch_target_angle = 0;

    get_motor_feedback();
    update_headings();
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
    gimbal.gimbal_act_mode = mode;
    gimbal.prev_gimbal_act_mode = mode;
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
    set_act_mode(act_mode);

    if (gimbal.gimbal_act_mode == SELF_GYRO ||
        gimbal.gimbal_act_mode == GIMBAL_FOLLOW) {
        set_motor_mode(GYRO_MODE);
    } else {
        set_motor_mode(ENCODE_MODE);
    }
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
                    Motors::parse_feedback(gimbal_can_ids[i],
                                           read_message.feedback[i],
                                           &(motor_controls[i].feedback));
                    break;
                }
            }
            // ASSERT(good == 0,
            //        "Gimbal motor ID is not provided in MOTOR_READ topic.");
        }
    }
}

/******************  MODE SELECTION FUNCTIONS BELOW ********************/
//void gimbal_get_raw_mpu_data(Gimbal_t *gbal, IMU_t *imu_hldr){
//	memcpy(&(gbal->ahrs_sensor), &(imu_hldr->ahrs_sensor), sizeof(AhrsSensor_t));
//}
//
///*
// * @brief     Copy the gyroscope data from imu and calculate quaternion
// * 			  and euler's angle through attitude-breakdown algorithms.
// * @param[in] gimbal: main gimbal handler
// * */
//void gimbal_get_euler_angle(Gimbal_t *gbal){
//	gimbal_get_raw_mpu_data(gbal, &imu); // copy data to avoid mem leaks
////	atti_math_calc(&gbal->ahrs_sensor, &gbal->euler_angle); //complementary filter parsed angle
////	mahony_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));	//mahony algo
//	madgwick_ahrs_update(&(gbal->ahrs_sensor), &(gbal->euler_angle));  //madgwick algo
//}

/*
 * @brief     Copy the gyroscope data from imu and calculate quaternion
 * 			  and euler's absolute angle through attitude-breakdown algorithms.
 * @param[in] gbal: main gimbal handler
 * */
void GimbalApp::update_imu_angle(float yaw, float pitch) {

    /* get timestamp */
    // uint32_t DWTcnt = dwt_getCnt_us();  // systemclock_core 168MHz ->usec
    //	 uint32_t delta_t = min(DWTcnt - gbal->euler_angle.timestamp, 3000);

    /* filter the yaw angle data to handle shift */
    gimbal.euler_angle.yaw =
        first_order_low_pass_filter(&(gimbal.folp_f_yaw), yaw);
    gimbal.euler_angle.pitch =
        first_order_low_pass_filter(&(gimbal.folp_f_pitch), pitch);
}
/*
 * @brief     update the relevant encoder angle
 * @param[in] gbal: main gimbal handler
 * */
//void gimbal_get_ecd_fb_data(Gimbal_t *gbal) {
//	Motor_Feedback_t motor_feedback_buffer[MOTOR_COUNT];
//	BaseType_t new_feedback_message = peek_message(MOTOR_READ, motor_feedback_buffer, 0);
//	if (new_feedback_message == pdTRUE) {
//		memcpy(&(gbal->yaw_ecd_fb), motor_feedback_buffer + GIMBAL_YAW_CAN_ID * 8, sizeof(Motor_Feedback_t));
//		gbal->yaw_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->yaw_ecd_fb.rx_angle, gbal->yaw_ecd_center);
//		memcpy(&(gbal->pitch_ecd_fb), motor_feedback_buffer + GIMBAL_PITCH_CAN_ID * 8, sizeof(Motor_Feedback_t));
//		gbal->pitch_ecd_fb.rx_angle = gimbal_get_ecd_rel_angle(gbal->pitch_ecd_fb.rx_angle, gbal->pitch_ecd_center);
//		gimbal_update_ecd_rel_angle(gbal);
//	}
//}

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
    int16_t pitch_ecd_rel_angle = GimbalApp::calc_ecd_rel_angle(
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

/******************  MODE SELECTION FUNCTIONS ABOVE ********************/

/*
 * @brief     set the target angle with limited range
 * @param[in] gbal: main gimbal handler
 * @param[in] target yaw and pitch relative angle(-pi, pi)
 */
void GimbalApp::set_limited_angle(float new_yaw_target_angle,
                                  float new_pitch_target_angle) {
    gimbal.yaw_target_angle = new_yaw_target_angle;
    gimbal.pitch_target_angle = new_pitch_target_angle;
    /* only set limit for yaw where is no slipring */
    //	VAL_LIMIT(gbal->yaw_tar_angle,
    //				   -PI,
    //				    PI);
    /* set the limit for pitch */
    VAL_LIMIT(gimbal.pitch_target_angle, -PITCH_GYRO_DELTA, PITCH_GYRO_DELTA);
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

void GimbalApp::send_rel_angles() {
    CANCommMessage_t rel_angle_message;
    rel_angle_message.topic_name = GIMBAL_REL_ANGLES;

    float temp_yaw = -gimbal.yaw_rel_angle;
    memcpy(rel_angle_message.data, &temp_yaw, sizeof(float));
    memcpy(&(rel_angle_message.data[4]), &(gimbal.pitch_rel_angle),
           sizeof(float));

    message_center.pub_message(COMM_OUT, &rel_angle_message);
}

void GimbalApp::update_headings() {
    if (gimbal.gimbal_motor_mode == GYRO_MODE) {
        gimbal.yaw_imu_angle = GimbalApp::calc_rel_angle(
            gimbal.yaw_imu_center, gimbal.euler_angle.yaw);
        gimbal.pitch_imu_angle = GimbalApp::calc_rel_angle(
            gimbal.pitch_imu_center, gimbal.euler_angle.pitch);

        gimbal.yaw_rel_angle = gimbal.yaw_imu_angle;
        gimbal.pitch_rel_angle = gimbal.pitch_imu_angle;
    } else if (gimbal.gimbal_motor_mode == ENCODE_MODE) {
        update_ecd_angles();

        gimbal.yaw_rel_angle = gimbal.yaw_ecd_angle;
        gimbal.pitch_rel_angle = gimbal.pitch_ecd_angle;
    } else {
        // TODO: Implement error handling / undefined motor mode state.
    }
}

void GimbalApp::update_targets(int16_t* g_channels) {
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
        }
    } else if (gimbal.gimbal_mode == PATROL_MODE &&
               gimbal.gimbal_act_mode == INDPET_MODE) {
        gimbal.yaw_target_angle = 0;
    } else {
        float deltas[2];
        calc_channels_to_angles(g_channels, deltas);

        gimbal.yaw_target_angle -= deltas[0];
        if (gimbal.yaw_target_angle > PI)
            gimbal.yaw_target_angle -= 2.0f * PI;
        if (gimbal.yaw_target_angle < -PI)
            gimbal.yaw_target_angle += 2.0f * PI;

        gimbal.pitch_target_angle -= deltas[1];
        if (gimbal.pitch_target_angle < -20.0f * DEGREE2RAD)
            gimbal.pitch_target_angle = -20.0f * DEGREE2RAD;
        if (gimbal.pitch_target_angle > 12.0f * DEGREE2RAD)
            gimbal.pitch_target_angle = 12.0f * DEGREE2RAD;
    }

    set_limited_angle(gimbal.yaw_target_angle, gimbal.pitch_target_angle);
}

//float gimbal_calc_dual_pid_out(PID2_t *f_pid, PID2_t *s_pid, float f_cur_val, s_cur_val) {
//	return pid2_dual_loop_control(f_pid,
//			s_pid,
//			0,
//			f_cur_val,
//			s_cur_val,
//			GIMBAL_TASK_EXEC_TIME * 0.001,
//			GIMBAL_TASK_EXEC_TIME * 0.001);
//}

/******************************** For Comms Above **************************************/
/******************************** For Auto Aiming Below ********************************/
// void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, UC_Auto_Aim_Pack_t *pack) {
//	float cur_yaw_target = 0.0;
//	float cur_pitch_target = 0.0;
//	float delta_yaw= 0.0;
//	float delta_pitch = 0.0;
//
//	if(gbal->gimbal_mode == PC_MODE){
//		/* filter applied here, TODO may add kalman filter here, depends on data input */
//		float filtered_delta_yaw = ewma_filter(&gbal->ewma_f_aim_yaw, pack->delta_yaw);
////		pack->yaw_data = sliding_window_mean_filter(&gbal->swm_f_aim_yaw, pack->yaw_data);
//		delta_yaw = in_out_map(filtered_delta_yaw, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI
//
//		float filtered_delta_pitch = ewma_filter(&gbal->ewma_f_aim_pitch, pack->delta_pitch);
////		pack->pitch_data = sliding_window_mean_filter(&gbal->swm_f_aim_pitch, pack->yaw_data);
//		delta_pitch = in_out_map(filtered_delta_pitch, -180.0, 180.0, -PI,PI);// 1000 -> 2*pi, old value +-30*PI
//	}
//	/* get the latest angle position of pitch and yaw motor */
//	gimbal_get_ecd_fb_data(&gimbal);
//	/* NOTE: Even if the target was beyond pi, the motor still tracked the same dir bc of spd loop and phase delay,
//	 * and right about next time, the feedback of motor would be changed from pi to -pi(or inverse), which will
//	 * also update the target into right scale of angle */
//	gimbal_update_rel_turns(gbal, GIMBAL_JUMP_THRESHOLD);
//	if(gbal->gimbal_motor_mode == GYRO_MODE){
////		 cur_yaw_target = gbal->yaw_cur_abs_angle - delta_yaw;
//		cur_yaw_target = gbal->final_abs_yaw - delta_yaw;
//		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
//		// cur_pitch_target = gbal->final_abs_pitch + delta_pitch * PITCH_GEAR_RATIO;
//	}
//	else{
////		cur_yaw_target = gbal->yaw_cur_rel_angle - delta_yaw;
//		cur_yaw_target = gbal->yaw_total_rel_angle - delta_yaw;
//		cur_pitch_target = gbal->pitch_cur_rel_angle + delta_pitch * PITCH_GEAR_RATIO;
//	}
//	/* avoid small noise to spin the yaw */
//	if(fabs(delta_yaw)>= 1.0f*DEGREE2RAD)
//		gbal->yaw_tar_angle = cur_yaw_target;
//	if(fabs(delta_pitch)>= 1.0f*DEGREE2RAD)
//		gbal->pitch_tar_angle = cur_pitch_target;
//	/* independent mode don't allow set yaw angle */
////	if(gbal->gimbal_act_mode == INDPET_MODE)
////		gbal->yaw_tar_angle = 0;
// }
/******************************** For Auto Aiming Above ********************************/

/*
 * @brief     Execute the cmd set by previous gimbal function. Usually the last called func.
 * @param[in] gbal: main gimbal handler
 * @param[in] mode: DUAL_LOOP_PID_CONTROL/SINGLE_LOOP_PID_CONTROL/GIMBAL_STOP
 * retval 	  None
 */
void GimbalApp::cmd_exec() {
    float yaw_diff = GimbalApp::calc_rel_angle(gimbal.yaw_target_angle,
                                               gimbal.yaw_rel_angle);
    float pitch_diff = GimbalApp::calc_rel_angle(gimbal.pitch_target_angle,
                                                 gimbal.pitch_rel_angle);

    pid2_dual_loop_control(
        &(motor_controls[GIMBAL_YAW_MOTOR_INDEX].f_pid),
        &(motor_controls[GIMBAL_YAW_MOTOR_INDEX].s_pid), 0, yaw_diff,
        motor_controls[GIMBAL_YAW_MOTOR_INDEX].feedback.rx_rpm,
        GIMBAL_TASK_EXEC_TIME * 0.001, GIMBAL_TASK_EXEC_TIME * 0.001);

    pid2_dual_loop_control(
        &(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].f_pid),
        &(motor_controls[GIMBAL_PITCH_MOTOR_INDEX].s_pid), 0, pitch_diff,
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

#endif /* __GIMBAL_APP_C__ */
