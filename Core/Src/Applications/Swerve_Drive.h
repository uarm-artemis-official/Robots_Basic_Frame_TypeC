#ifndef __SWERVE_DRIVE_H
#define __SWERVE_DRIVE_H

#include <array>
#include "apps_interfaces.h"
#include "apps_types.h"
#include "subsystems_interfaces.h"

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    IMessageCenter& message_center;
    uint32_t width;
    float target_wheel_angles[4];
    int16_t target_wheel_speeds[4];
    std::array<Swerve_Wheel_Control_t, 8> swerve_motors;

   public:
    static int32_t pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                         uint32_t angle);

    SwerveDrive(IMessageCenter& message_center_ref, uint32_t chassis_width);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

#endif