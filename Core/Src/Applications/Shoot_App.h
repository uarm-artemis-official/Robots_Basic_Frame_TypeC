/*******************************************************************************
* @file           : Shoot_App.h
* @brief          : the shooting task handling fric and magazine motor
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SHOOT_APP_H__
#define __SHOOT_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class ShootApp : public RTOSApp<ShootApp> {
   private:
    IMessageCenter& message_center;
    IAmmoLid& ammo_lid;
    IMotors& motors;

    Shoot shoot;
    LoaderControl loader_control;
    FlyWheelControl flywheel_controls[2];
    const float LOADER_ACTIVE_RPM;
    const float FLYWHEEL_ACTIVE_TARGET_RPM;
    const float MAX_FLYWHEEL_ACCEL;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = SHOOT_TASK_EXEC_TIME;

    ShootApp(IMessageCenter& message_center_ref, IAmmoLid& ammo_lid_ref,
             IMotors& motors_ref, float loader_active_rpm_,
             float flywheel_target_rpm_, float max_flywheel_accel);

    void init();
    void loop();

    void get_motor_feedback();
    void detect_loader_stall();

    void process_commands();

    void calc_targets();
    void calc_motor_outputs();
    void send_motor_outputs();

    void set_shoot_mode(ShootActMode_t new_mode);
    void set_loader_target(float new_target);
    void set_flywheel_target(float new_target);
};

#endif /* __SHOOT_APP_H__ */
