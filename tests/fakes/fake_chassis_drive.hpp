#ifndef FAKE_CHASSIS_DRIVE_HPP
#define FAKE_CHASSIS_DRIVE_HPP

#include "apps_interfaces.hpp"

class FakeChassisDrive : public ChassisDrive<FakeChassisDrive> {
   public:
    float mock_power_consumption = 0.0f;
    float last_vx = 0.0f;
    float last_vy = 0.0f;
    float last_wz = 0.0f;

    bool init_impl() { return true; }

    void drive_impl(float vx, float vy, float wz) {
        last_vx = vx;
        last_vy = vy;
        last_wz = wz;
    }

    void get_motor_feedback() {}

    void calc_motor_outputs(float vx, float vy, float wz) {
        last_vx = vx;
        last_vy = vy;
        last_wz = wz;
    }

    void send_motor_messages() {}

    float calc_power_consumption() { return mock_power_consumption; }

    void set_max_power_impl(float new_max_power) { (void) new_max_power; }

    void set_mock_power_consumption(float new_mock_power) {
        mock_power_consumption = new_mock_power;
    }
};

#endif  // FAKE_CHASSIS_DRIVE_HPP
