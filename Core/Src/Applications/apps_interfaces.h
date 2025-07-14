#ifndef __APPS_INTERFACES_H
#define __APPS_INTERFACES_H

#include "uarm_os.h"

// TODO: Add startup events for apps so certain apps will start after certain events.
// e.g. Chassis and Gimbal Apps only start when motors are detected to be online.
// TODO: Add another template parameter for task period with getter function to access it.
template <class Derived>
class ExtendedRTOSApp {
   public:
    void run(const void* argument) {
        // TODO: Implement static asserts to check if derived has the following defined:
        //  - Derived::LOOP_PERIOD_MS
        //  - void init()
        //  - void calibrate()
        //  - bool exit_calibrate_cond()
        //  - void after_calibrate()
        //  - bool exit_loop_prepare_cond()
        //  - void loop_prepare()
        //  - void after_loop_prepare()
        //  - void loop()

        (void) argument;
        Derived* derived = static_cast<Derived*>(this);
        TickType_t xLastWakeTime;
        static_assert(Derived::LOOP_PERIOD_MS != 0);
        const TickType_t xFrequency = pdMS_TO_TICKS(Derived::LOOP_PERIOD_MS);
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

template <class Derived>
class RTOSApp : public ExtendedRTOSApp<Derived> {
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