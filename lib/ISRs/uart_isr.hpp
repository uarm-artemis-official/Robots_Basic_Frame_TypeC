#ifndef __UART_ISR_HPP
#define __UART_ISR_HPP

#include <functional>
#include "isr_interfaces.hpp"
#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace isr {
    namespace uart {
        enum class ECallbacks { RECEIVE_COMPLETE, ON_ERROR, TRANSMIT_COMPLETE };

        using TISRState = MW_UART::Peripheral;
        using TInitFunc = std::function<bool(MW_UART::IUART&)>;
        using TISRRoutine =
            std::function<void(MW_UART::IUART&, MW_UART::Peripheral)>;

        // TODO: Add resource reservation to avoid conflicts.
        class UART_ISR
            : public ISR<ECallbacks, TISRRoutine, TInitFunc, TISRState> {
           private:
            MW_UART::IUART& uart;

           public:
            explicit UART_ISR(MW_UART::IUART& uart_ref) : uart(uart_ref) {}

            [[nodiscard]] bool init() override {
                initialized = true;
                return true;
            }

            [[nodiscard]] bool on_register_init(size_t init_func_idx) override {
                return init_funcs[init_func_idx](uart);
            }

            [[nodiscard]] bool on_register_routine(
                size_t routine_func_idx) override {
                (void) routine_func_idx;
                return true;
            }

            void run_isr_routines(ECallbacks callback_running,
                                  TISRState callback_state) override {
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
        };

        bool install_isr(UART_ISR& uart_isr_ref);
    }  // namespace uart
}  // namespace isr

#endif