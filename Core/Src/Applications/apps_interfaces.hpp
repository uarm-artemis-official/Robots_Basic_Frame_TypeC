#ifndef __APPS_INTERFACES_H
#define __APPS_INTERFACES_H

#include "uarm_os.hpp"

// TODO: Add startup events for apps so certain apps will start after certain events.
// e.g. Chassis and Gimbal Apps only start when motors are detected to be online.
// TODO: Add another template parameter for task period with getter function to access it.
template <class Derived, uint32_t _loop_period_ms>
class ExtendedRTOSApp {
   public:
    static_assert(_loop_period_ms > 0,
                  "There must be a delay between loops for RTOS apps.");
    static constexpr uint32_t loop_period_ms = _loop_period_ms;
    static constexpr float get_loop_period() {
        return static_cast<float>(loop_period_ms) / 1000;
    }

    void run(const void* argument) {
        (void) argument;
        Derived* derived = static_cast<Derived*>(this);
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = pdMS_TO_TICKS(loop_period_ms);
        xLastWakeTime = xTaskGetTickCount();
        derived->init();
        for (;;) {
            if (derived->exit_calibrate_cond()) {
                break;
            } else {
                derived->calibrate();
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        for (;;) {
            derived->loop();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }
};

template <class Derived, int loop_period_ms>
class RTOSApp : public ExtendedRTOSApp<Derived, loop_period_ms> {
   public:
    void calibrate() {}
    bool exit_calibrate_cond() { return true; }
};

// TODO Make policies sub-folder and move ChassisDrive into that folder.
template <class Derived>
class ChassisDrive {
   public:
    void init() {
        Derived* derived = static_cast<Derived*>(this);
        derived->init_impl();
    }

    void drive(float vx, float vy, float wz) {
        Derived* derived = static_cast<Derived*>(this);
        derived->get_motor_feedback();
        derived->calc_motor_outputs(vx, vy, wz);
        derived->send_motor_messages();
    }

    float get_power_consumption() {
        Derived* derived = static_cast<Derived*>(this);
        return derived->calc_power_consumption();
    }

    void set_max_power(float new_max_power) {
        Derived* derived = static_cast<Derived*>(this);
        derived->set_max_power_impl(new_max_power);
    }
};

#endif