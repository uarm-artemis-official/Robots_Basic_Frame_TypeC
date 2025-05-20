/*******************************************************************************
* @file           : Chassis_App.h
* @brief          : chassis task managing 4 chassis motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __CHASSIS_APP_H__
#define __CHASSIS_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class ChassisApp : public RTOSApp<ChassisApp> {
   private:
    IMessageCenter& message_center;
    IDebug& debug;

    Chassis_t chassis;
    Chassis_Wheel_Control_t motor_controls[CHASSIS_MAX_WHEELS];
    int16_t rc_channels[4];

   public:
    // static constexpr uint32_t LOOP_PERIOD_MS = CHASSIS_TASK_EXEC_TIME;

    ChassisApp(IMessageCenter& message_center_ref, IDebug& debug_ref);
    void init();
    void set_initial_state();

    void loop();

    void mecanum_wheel_calc_speed();
    void chassis_update_chassis_coord(int16_t* channels);
    void chassis_update_gimbal_coord(int16_t* channels);
    void chassis_brake(float* vel, float ramp_step, float stop_threshold);
    void chassis_exec_act_mode();

    void chassis_calc_wheel_pid_out();
    void chassis_get_rc_info(int16_t* channels);
    void chassis_get_wheel_feedback();
    void chassis_get_gimbal_rel_angles();

    void chassis_send_wheel_volts();

    /* power limit */
    //void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t *ref);
    void chassis_power_limit_referee();
    void chassis_power_limit_local(uint16_t local_power_limit);
    void select_chassis_speed(uint8_t level);
    //void chassis_manual_gear_set(Chassis_t* chassis_hdlr, RemoteControl_t *rc_hdlr);
};

#endif /* SRC_APPLICATIONS_CHASSIS_APP_H_ */
