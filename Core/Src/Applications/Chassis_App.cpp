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

    pid2_init(chassis.spin_pid, robot_config::pid_params::KP_CHASSIS_SPIN,
              robot_config::pid_params::KI_CHASSIS_SPIN,
              robot_config::pid_params::KD_CHASSIS_SPIN,
              robot_config::pid_params::BETA_CHASSIS_SPIN,
              robot_config::pid_params::YETA_CHASSIS_SPIN,
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
    chassis.max_vx = ChassisApp::MAX_TRANSLATION;
    chassis.max_vy = ChassisApp::MAX_TRANSLATION;
    chassis.max_wz = ChassisApp::MAX_ROTATION;

    chassis.v_perp = 0;
    chassis.v_parallel = 0;

    chassis.gimbal_yaw_rel_angle = 0;

    memset(&(chassis.ref_power_stat), 0, sizeof(ChassisPowerStat_t));
    memset(rc_channels, 0, sizeof(int16_t) * 4);
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::loop() {
    chassis_get_rc_info(rc_channels);
    chassis_get_gimbal_rel_angles();

    update_movement_commands();
    calc_movement_vectors();

    drive_train.drive(chassis.vx, chassis.vy, chassis.wz);
}

template <class DriveTrain>
void ChassisApp<DriveTrain>::update_movement_commands() {
    // TODO: Replace 660 with constant describing max rc channel.
    chassis.v_perp = in_out_map(rc_channels[2], -660, 660, -MAX_TRANSLATION,
                                MAX_TRANSLATION);
    chassis.v_parallel = in_out_map(rc_channels[3], -660, 660, -MAX_TRANSLATION,
                                    MAX_TRANSLATION);
    chassis.wz =
        in_out_map(rc_channels[0], -660, 660, -MAX_ROTATION, MAX_ROTATION);
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
            chassis.wz = GYRO_SPEED;
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

/*
 * @brief brake the chassis slowly to avoid instant power overlimt
 */
template <class DriveTrain>
void ChassisApp<DriveTrain>::chassis_brake(float* vel, float ramp_step,
                                           float stop_threshold) {
    if (*vel > 0)           // both release -> brake
        *vel -= ramp_step;  //brake need to be quicker
    else if (*vel < 0)
        *vel += ramp_step;
    if (fabs(*vel) < stop_threshold)
        *vel = 0;
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
void ChassisApp<DriveTrain>::chassis_get_rc_info(int16_t* channels) {
    RCInfoMessage_t rc_info;
    BaseType_t new_message = message_center.peek_message(RC_INFO, &rc_info, 0);

    if (new_message == pdTRUE) {
        // TODO: Add input validation for modes and channels.
        BoardMode_t board_mode = static_cast<BoardMode_t>(rc_info.modes[0]);
        BoardActMode_t act_mode = static_cast<BoardActMode_t>(rc_info.modes[1]);

        chassis.chassis_mode = board_mode;
        chassis.chassis_act_mode = act_mode;
        memcpy(channels, &(rc_info.channels), sizeof(int16_t) * 4);
    }
}

#ifdef CHASSIS_POWER_LIMIT
/******************************************************************************************************************
 * CHASSIS POWER MANAGEMENT PROCESS WALKTHROUGH
 ******************************************************************************************************************
 * 	@attention: rules:
 *  The chassis power consumption of robots will be continuously monitored by the Referee System, and the robot
	chassis needs to run within the chassis power consumption limit. Considering it is difficult for a robot to control
	instantaneous output power when in motion, a buffer energy (Z) has been defined to avoid the consequent penalty.
 *  The buffer energy value of Hero, Standard and Sentry Robots is 60J.
 *  Excess Percentage: K = (Pr－Pl) / Pl * 100%, where Pr is the instantaneous Chassis Power Consumption output and
	Pl is the power consumption limit.
 *	W = Pt, meaning that if we running infantry with power=100w (the limit of the power=40w, level 1 infantry), after 1s
 	Hp would be deducted.(t = (100-40)/buffer_energy)

 	Power limit:(RMNA 2023 Rules manual)							Buffer energy:60J
 				Mode			Power-focused		HP-focues
 	infantry  level 1				 60                 45
 			  level 2			     80					50
 			  level 3   			100					55

 Strategy:
 * 1) Decrease the current value of each Mecanum wheel in equal proportions so that the total power does not exceed
 * 	  the power limit
 ******************************************************************************************************************/

/*
 * @brief     the power management of chassis strategy 1: proportionally decrease based on referee callback.
 * @param[in] chassis: main chassis handler
 * */
void chassis_power_limit_referee(Chassis_t* chassis_hdlr) {
    int32_t total_current = 0;
    int32_t abs_current_weighted_sum = 0;
    //step 1： Read chassis power limit rx data from referee system (op: combine super capacity power buffer)
    get_chassis_ref_power_stat(chassis_hdlr, &referee);
    //step 2: Converts the power limit to the total maximum motor output current(threshold)

    //step 3: Compare the threshold with a DANGER value, determine the power limit ratio (possibly)
    /**************************************** Important FIXME **********************************************/
    //FIXME: We may need to consider using the current feedback to more precisely calc total current thus total
    //		 power(Pt = Sum(motor[i] for i=range(0:3)) * U(24v)), not simply hard-set - it may cause significant shift
    //		 if needed, we may also apply pid control for it.
    //FIXME: The condition here is too naive and not considering the sampling time
    //		 the critical condition should be like: W_buffer - W_danger = samplingTime * P_limit
    /**************************************** Important FIXME **********************************************/
    if (chassis_hdlr->ref_power_stat.power >=
        CHASSIS_POWER_THRESHOLD) {  // FIXME: here we need to change the threshold
        //based on the robots level feedback, refer to public defines
        // stop chassis immediately
        total_current = 0;
    }
    if (chassis_hdlr->ref_power_stat.power >= CHASSIS_POWER_DANGER) {

        // decrease the chassis spd immediately
        total_current = CHASSIS_POWER_DANGER *
                        POWER_TO_CURRENT;  //random value, need to test
    }
    //step 4: Weighted assignment (soft-max filter) of the current motor output value (conditional judgment)
    //softmax(chassis_hdlr->mec_spd, 4); //softmax cannot handle negative value very well
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        chassis_hdlr->mec_spd[i] =
            chassis_hdlr->mec_spd[i] *
            (abs(chassis_hdlr->mec_spd[i]) / abs_current_weighted_sum);
}

/*
 * @brief     the power management of chassis strategy 2: proportionally decrease based on local motor data.
 * @param[in] chassis: main chassis handler
 * @param[in] local_power_limit
 *
 * @note: this function calculates the power based on the equation below:
 * 			P = UI, where the voltage U is 24V, and the current can be obtained from can feedbcak data.
 * */
void chassis_power_limit_local(Chassis_t* chassis_hdlr,
                               uint16_t local_power_limit) {
    /* step 1: get the current power */
    int16_t current_power = 0;
    int32_t total_current = 0;
    int32_t abs_current_weighted_sum = 0;

    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
        current_power +=
            24 * motor_data[i].motor_feedback.rx_current * CHASSIS_MAX_SPEED /
            20;  //assume the feedbcak current has same range with tx data
    }

    /* check if the power over danger zone */
    if (current_power >=
        CHASSIS_POWER_THRESHOLD) {  //assume we don't have a power feedbcak, need to manually set limit
        total_current = 0;
    } else if (current_power >= local_power_limit) {
        total_current = (int32_t) local_power_limit / 24;
    }

    /*apply current */
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        chassis_hdlr->mec_spd[i] =
            (int16_t) chassis_hdlr->mec_spd[i] *
            (abs(chassis_hdlr->mec_spd[i]) / abs_current_weighted_sum);
}

/* get the latest current power and power limit from referee system */
void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t* ref) {
    chassis_hdlr->ref_power_stat.current = ref->power_heat_data.chassis_current;
    chassis_hdlr->ref_power_stat.power = ref->power_heat_data.chassis_power;
    chassis_hdlr->ref_power_stat.buffer_energy =
        ref->power_heat_data.buffer_energy;
}
#endif

/* only for sentry begin */