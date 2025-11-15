#include "uart_isr.hpp"
#include "uarm_lib.hpp"

namespace isr {
    namespace uart {
        UART_ISR::UART_ISR(MW_UART::IUART& _uart) : uart(_uart) {}

        // TODO: Add resource reservation to avoid conflicts.
        bool UART_ISR::init() {
            bool success = true;
            for (size_t i = 0; i < init_funcs_size; i++) {
                success &= init_funcs[i](uart);
            }
            return success;
        }

        void UART_ISR::run_isr_routines(ECallbacks callback_running,
                                        TISRState callback_state) {
            switch (callback_running) {
                case ECallbacks::RECEIVE_COMPLETE: {
                    for (size_t i = 0; i < routines_size; i++) {
                        if (routines[i].second == callback_running) {
                            routines[i].first(uart, callback_state);
                        }
                    }
                    break;
                }
                case ECallbacks::ON_ERROR: {
                    for (size_t i = 0; i < routines_size; i++) {
                        if (routines[i].second == callback_running) {
                            routines[i].first(uart, callback_state);
                        }
                    }
                    break;
                }
                default:
                    ASSERT(false, "Unhandled CAN ISR callback.");
                    break;
            }
        }
    }  // namespace uart
}  // namespace isr