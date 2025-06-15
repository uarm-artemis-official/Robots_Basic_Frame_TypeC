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
#include "uarm_math.h"

template <class DriveTrain>
class ChassisApp : public RTOSApp<ChassisApp<DriveTrain>> {
   private:
    ChassisDrive<DriveTrain>& drive_train;
    IMessageCenter& message_center;
    IDebug& debug;

    Chassis_t chassis;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = CHASSIS_TASK_EXEC_TIME;
    static constexpr float MAX_TRANSLATION = 2;  // in m/s
    static constexpr float MAX_ROTATION = PI;    // rad/s
    static constexpr float GYRO_SPEED = PI;

    ChassisApp(DriveTrain& drive_train_ref, IMessageCenter& message_center_ref,
               IDebug& debug_ref);
    void init();
    void set_initial_state();

    void loop();

    void calc_movement_vectors();
    void chassis_brake(float* vel, float ramp_step, float stop_threshold);

    void chassis_get_rc_info();
    void process_commands();
    void chassis_get_gimbal_rel_angles();

    /* power limit */
    //void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t *ref);
    void chassis_power_limit_referee();
    void chassis_power_limit_local(uint16_t local_power_limit);
    void select_chassis_speed(uint8_t level);
    //void chassis_manual_gear_set(Chassis_t* chassis_hdlr, RemoteControl_t *rc_hdlr);
};

#endif /* SRC_APPLICATIONS_CHASSIS_APP_H_ */
