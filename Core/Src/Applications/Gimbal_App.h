/*******************************************************************************
* @file           : Gimbal_App.h
* @brief          : gimbal task managing 2 gimbal motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __GIMBAL_APP_H__
#define __GIMBAL_APP_H__

#include <array>
#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

/*
 * @attention:
 * 		We transit each angle mentioned here to radian because:
 * 		1. Unit and normalize the calculation of PID (spec for angular loop)
 * 		2. for safety consideration, sometimes swap the mode between gyro and ecd
 * 		   keep radians helping unit the input value
 * 		3. easier for calculating cos/sin functions
 *
 * */
class GimbalApp : public ExtendedRTOSApp<GimbalApp> {
   private:
    Gimbal_t gimbal;
    Gimbal_Imu_Calibration_t imu_calibration;
    Gimbal_Motor_Control_t motor_controls[GIMBAL_MOTOR_COUNT];
    int16_t gimbal_channels[2];

    IMessageCenter& message_center;
    IEventCenter& event_center;
    IDebug& debug;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = GIMBAL_TASK_EXEC_TIME;
    static float calc_rel_angle(float angle1, float angle2);
    static int16_t calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);

    GimbalApp(IMessageCenter& message_center_ref, IEventCenter& event_center,
              IDebug& debug_ref);
    void init();
    void set_initial_state();

    bool exit_calibrate_cond();
    void calibrate();

    void after_calibrate();
    bool exit_loop_prepare_cond();
    void loop_prepare();
    void after_loop_prepare();

    void loop();

    void set_modes(uint8_t modes[3]);
    void set_board_mode(BoardMode_t mode);
    void set_act_mode(BoardActMode_t mode);
    void set_motor_mode(GimbalMotorMode_t mode);

    void get_rc_info();
    void get_motor_feedback();
    void get_imu_headings();

    void safe_mode_switch();
    void get_euler_angle();
    void update_imu_angle(float yaw, float pitch);
    void update_ecd_euler_angle(float yaw_target_angle,
                                float pitch_target_angle);
    void update_ecd_angles();
    void update_truns(float halfc);
    void set_angle(float target_angle);
    void set_limited_angle(float yaw_target_angle, float pitch_target_angle);
    void set_spd(int16_t yaw_target_spd);
    void cmd_exec();
    void update_rel_turns(int jump_threshold);
    void send_rel_angles();
    void calc_rel_targets(float delta_yaw, float delta_pitch);
    void calc_channels_to_angles(const int16_t g_channels[2], float deltas[2]);
    // void gimbal_update_autoaim_rel_angle(Gimbal_t *gbal, UC_Auto_Aim_Pack_t *pack);
    float calc_dual_pid_out(PID2_t* f_pid, PID2_t* s_pid, float f_cur_val);
    void send_motor_volts();

    void update_headings();
    void update_targets(int16_t* g_channels);
};

#endif /* __SRC_APPLICATIONS_GIMBAL_APP_H_ */