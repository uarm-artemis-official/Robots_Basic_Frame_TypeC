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

namespace shoot_app {
    class ShootApp : public RTOSApp<ShootApp> {
       private:
        IMessageCenter& message_center;
        IAmmoLid& ammo_lid;

        Shoot shoot;
        LoaderControl loader_control;
        FlyWheelControl flywheel_controls[2];
        const float loader_rpm;

       public:
        static constexpr uint32_t LOOP_PERIOD_MS = SHOOT_TASK_EXEC_TIME;
        static constexpr float FLYWHEEL_TARGET_RPM = 10.0;

        ShootApp(IMessageCenter& message_center_ref, IAmmoLid& ammo_lid_ref,
                 float loader_rpm_);

        void init();
        void loop();

        void get_rc_info();
        void get_motor_feedback();

        void calc_targets();
        void calc_motor_outputs();
        void send_motor_outputs();

        void set_shoot_mode(ShootActMode_t new_mode);
    };
}  // namespace shoot_app

#endif /* __SHOOT_APP_H__ */
