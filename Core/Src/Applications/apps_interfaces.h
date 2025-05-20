#ifndef __APPS_INTERFACES_H
#define __APPS_INTERFACES_H

#include "uarm_os.h"

// TODO: Implement CRTP for app that has init, calibrate, and loop stages.
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
        //  - after_loop_prepare()
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

        derived->after_calibrate();
        for (;;) {
            if (derived->exit_loop_prepare_cond()) {
                break;
            } else {
                derived->loop_prepare();
            }
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        derived->after_loop_prepare();

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

    void after_calibrate() {}

    bool exit_loop_prepare_cond() { return true; }
    void loop_prepare() {}
    void after_loop_prepare() {}
};

#endif