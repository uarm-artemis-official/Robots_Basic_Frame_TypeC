#ifndef __OMNI_DRIVE_H
#define __OMNI_DRIVE_H

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class OmniDrive : public ChassisDrive<OmniDrive> {
   private:
    Chassis_Wheel_Control_t motor_controls[CHASSIS_MAX_WHEELS];
    int16_t motor_angluar_vel[4];
    IMessageCenter& message_center;
    float width, length, max_power;

   public:
    OmniDrive(IMessageCenter& message_center_ref, float chassis_width,
              float chassis_length);

    void init_impl();

    void get_motor_feedback();

    void calc_target_motor_speeds(float vx, float vy, float wz);
    void calc_motor_volts();
    void calc_motor_outputs(float vx, float vy, float wz);
    float calc_power_consumption();

    void send_motor_messages();
};

#endif