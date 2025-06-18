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
    float command_deltas[2];

    IMessageCenter& message_center;
    IEventCenter& event_center;
    IDebug& debug;
    IMotors& motors;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = GIMBAL_TASK_EXEC_TIME;

    // Software limits on pitch targets to prevent pitch from hitting mechanical hard stops.
    static constexpr float PITCH_LOWER_LIMIT = -0.1;
    static constexpr float PITCH_UPPER_LIMIT = 0.4;
    static constexpr uint32_t IMU_CENTER_TARGET_SAMPLES = 100;

    static float calc_rel_angle(float angle1, float angle2);
    static int16_t calc_ecd_rel_angle(int16_t raw_ecd, int16_t center_offset);

    GimbalApp(IMessageCenter& message_center_ref, IEventCenter& event_center,
              IDebug& debug_ref, IMotors& motors_ref);
    void init();
    void set_initial_state();
    void wait_for_motors();

    bool exit_calibrate_cond();
    void calibrate();

    void loop();

    bool is_imu_calibrated();

    void set_modes(uint8_t modes[3]);
    void set_board_mode(BoardMode_t mode);
    void set_act_mode(BoardActMode_t mode);
    void set_motor_mode(GimbalMotorMode_t mode);
    void safe_mode_switch();

    void get_motor_feedback();
    void get_imu_headings();

    void process_commands();

    void calc_channels_to_angles(const int16_t g_channels[2], float deltas[2]);
    void calc_imu_center();

    void update_imu_angle(float yaw, float pitch);
    void update_ecd_angles();
    void update_headings();
    void update_targets();

    void cmd_exec();

    void send_motor_volts();
    void send_rel_angles();
};

#endif /* __SRC_APPLICATIONS_GIMBAL_APP_H_ */