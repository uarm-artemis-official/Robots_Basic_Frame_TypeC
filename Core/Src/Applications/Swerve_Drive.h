#ifndef __SWERVE_DRIVE_H
#define __SWERVE_DRIVE_H

#include <array>
#include "apps_interfaces.h"
#include "apps_types.h"
#include "subsystems_interfaces.h"

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    static constexpr size_t DRIVE_MOTOR1_INDEX = 0;
    static constexpr size_t DRIVE_MOTOR2_INDEX = 1;
    static constexpr size_t DRIVE_MOTOR3_INDEX = 2;
    static constexpr size_t DRIVE_MOTOR4_INDEX = 3;
    static constexpr size_t STEER_MOTOR1_INDEX = 4;
    static constexpr size_t STEER_MOTOR2_INDEX = 5;
    static constexpr size_t STEER_MOTOR3_INDEX = 6;
    static constexpr size_t STEER_MOTOR4_INDEX = 7;

    static constexpr size_t OUTPUT1_INDEX = 0;
    static constexpr size_t OUTPUT2_INDEX = 1;
    static constexpr size_t OUTPUT3_INDEX = 2;
    static constexpr size_t OUTPUT4_INDEX = 3;

    IMessageCenter& message_center;
    const float width;

    std::array<Swerve_Wheel_Control_t, 8> swerve_motors;

    float steer_target_angle[4];
    uint16_t steer_max_speed[4];
    bool steer_ccw[4];

    int16_t drive_target_rpm[4];

   public:
    static int32_t pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                         uint32_t angle);

    SwerveDrive(IMessageCenter& message_center_ref, float chassis_width);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

#endif