#ifndef __TIMER_SERVICE_HPP
#define __TIMER_SERVICE_HPP

#include <array>
#include <cstddef>
#include <functional>

#include "middleware_interfaces.hpp"

namespace services {
    namespace timer {

        constexpr size_t MAX_TIMERS = 10;
        constexpr MW_RTOS::TimerHandle NULL_TIMER_HANDLE = nullptr;

        using CallbackFunc = std::function<void()>;

        extern MW_RTOS::TimerCallback on_timer_callback_cfunc;

        struct TimerServiceState {
            std::array<MW_RTOS::TimerHandle, MAX_TIMERS> timers {};
            std::array<CallbackFunc, MAX_TIMERS> callbacks {};
            size_t timers_count = 0;
            bool initialized = false;
        };

        class TimerService {
           private:
            TimerServiceState state;
            MW_RTOS::IRTOS& rtos;

           public:
            explicit TimerService(MW_RTOS::IRTOS& rtos_ref) : rtos(rtos_ref) {}

            bool init() {
                state.initialized = true;
                return true;
            }

            bool timer_create(MW_RTOS::TimerHandle& timer_ref,
                              const char* timer_name, uint32_t duration_ms,
                              MW_RTOS::TimerMode mode, CallbackFunc callback) {
                if (state.timers_count >= MAX_TIMERS) {
                    return false;
                }

                rtos.timer_create(timer_ref, timer_name, duration_ms, mode,
                                  on_timer_callback_cfunc);
                if (timer_ref == nullptr) {
                    return false;
                }

                for (size_t i = 0; i < MAX_TIMERS; i++) {
                    if (state.timers[i] == NULL_TIMER_HANDLE) {
                        state.timers[i] = timer_ref;
                        state.callbacks[i] = callback;
                        state.timers_count++;
                        return true;
                    }
                }

                return false;
            }

            bool timer_start(MW_RTOS::TimerHandle timer) {
                return rtos.timer_start(timer);
            }

            bool timer_start_from_isr(MW_RTOS::TimerHandle timer,
                                      bool* awaken_higher_prio) {
                return rtos.timer_start_from_isr(timer, awaken_higher_prio);
            }

            bool timer_reset(MW_RTOS::TimerHandle timer) {
                return rtos.timer_reset(timer);
            }

            bool timer_stop(MW_RTOS::TimerHandle timer) {
                return rtos.timer_stop(timer);
            }

            bool timer_stop_from_isr(MW_RTOS::TimerHandle timer,
                                     bool* awaken_higher_prio) {
                return rtos.timer_stop_from_isr(timer, awaken_higher_prio);
            }

            bool timer_delete(MW_RTOS::TimerHandle timer) {
                bool timer_deleted = rtos.timer_delete(timer);
                if (timer_deleted) {
                    for (size_t i = 0; i < MAX_TIMERS; i++) {
                        if (state.timers[i] == timer) {
                            state.timers[i] = NULL_TIMER_HANDLE;
                            state.callbacks[i] = nullptr;
                            state.timers_count--;
                            break;
                        }
                    }
                    return true;
                }
                return false;
            }

            void on_timer_callback(MW_RTOS::TimerHandle timer) {
                for (size_t i = 0; i < MAX_TIMERS; i++) {
                    if (state.timers[i] == timer) {
                        if (state.callbacks[i]) {
                            state.callbacks[i]();
                        }
                        break;
                    }
                }
            }

            TimerServiceState get_state() const { return state; }
            bool is_initialized() const { return state.initialized; }
        };

    }  // namespace timer
}  // namespace services

#endif