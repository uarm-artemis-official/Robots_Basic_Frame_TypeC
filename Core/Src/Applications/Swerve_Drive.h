#ifndef __SWERVE_DRIVE_H
#define __SWERVE_DRIVE_H

#include <array>
#include "apps_interfaces.h"
#include "apps_types.h"
#include "subsystems_interfaces.h"

class SwerveDrive : public ChassisDrive<SwerveDrive> {
   private:
    static constexpr size_t NUM_STEER_MOTORS = 4;
    static constexpr size_t NUM_DRIVE_MOTORS = 4;

    IMessageCenter& message_center;
    const float width;
    const float dt;

    std::array<Swerve_Drive_Control_t, NUM_DRIVE_MOTORS> drive_motors;
    std::array<Swerve_Steer_Control_t, NUM_STEER_MOTORS> steer_motors;

    std::array<float, NUM_STEER_MOTORS> steer_curr_angle;
    std::array<int16_t, NUM_STEER_MOTORS> steer_curr_speed;
    std::array<float, NUM_STEER_MOTORS> steer_cw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_ccw_mag;
    std::array<float, NUM_STEER_MOTORS> steer_target_angle;
    std::array<float, NUM_DRIVE_MOTORS> drive_target_speed;

    std::array<uint16_t, NUM_STEER_MOTORS> steer_max_speed;
    std::array<bool, NUM_STEER_MOTORS> steer_ccw;
    std::array<float, NUM_STEER_MOTORS> steer_output_angle;
    std::array<int32_t, NUM_DRIVE_MOTORS> drive_output;

   public:
    static int32_t pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                         uint32_t angle);

    explicit SwerveDrive(IMessageCenter& message_center_ref, float width_,
                         float dt_);

    void init_impl();
    void get_motor_feedback();
    void calc_motor_outputs(float vx, float vy, float wz);
    void send_motor_messages();
    float calc_power_consumption();
};

#endif