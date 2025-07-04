#ifndef __OMNI_DRIVE_H
#define __OMNI_DRIVE_H

#include <array>
#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class OmniDrive : public ChassisDrive<OmniDrive> {
   private:
    std::array<Chassis_Wheel_Control_t, 4> motor_controls;
    std::array<float, 4> motor_angular_vel;
    std::array<float, 4> wheel_power_consumption;
    IMessageCenter& message_center;
    IMotors& motors;
    float width, length, power_limit, chassis_dt;
    const float a = 0;
    const float k1 = 0;
    const float k2 = 0;

   public:
    OmniDrive(IMessageCenter& message_center_ref, IMotors& motors,
              float chassis_width, float chassis_length, float power_limit_,
              float chassis_dt_);

    void init_impl();

    void get_motor_feedback();

    void calc_target_motor_speeds(float vx, float vy, float wz);
    void calc_motor_volts();
    void calc_motor_outputs(float vx, float vy, float wz);
    void calc_power_limits();

    void calc_wheel_power_consumption();
    float calc_power_consumption();
    void set_max_power_impl(float new_max_power);

    void send_motor_messages();
};

#endif