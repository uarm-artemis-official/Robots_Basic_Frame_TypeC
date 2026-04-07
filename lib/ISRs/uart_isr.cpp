#include "uart_isr.hpp"
#include "uarm_lib.hpp"

namespace isr {
    namespace uart {
        UART_ISR::UART_ISR(MW_UART::IUART& _uart) : uart(_uart) {}

        bool UART_ISR::init() {
            return true;
        }

        void UART_ISR::run_isr_routines(ECallbacks callback_running,
                                        TISRState callback_state) {
            switch (callback_running) {
                case ECallbacks::RECEIVE_COMPLETE:
                    [[fallthrough]];
                case ECallbacks::ON_ERROR:
                    [[fallthrough]];
                case ECallbacks::TRANSMIT_COMPLETE:
                    break;
                default:
                    ASSERT(false, "Unhandled UART ISR callback.");
                    break;
            }

            for (size_t i = 0; i < routines_size; i++) {
                if (routines[i].second == callback_running) {
                    routines[i].first(uart, callback_state);
                }
            }
        }

        bool UART_ISR::on_register_init(size_t init_func_idx) {
            return init_funcs[init_func_idx](uart);
        }

        bool UART_ISR::on_register_routine(size_t routine_func_idx) {
            (void) routine_func_idx;
            return true;
        }
    }  // namespace uart
}  // namespace isr