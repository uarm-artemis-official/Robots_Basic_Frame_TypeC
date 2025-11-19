#ifndef __UART_ISR_HPP
#define __UART_ISR_HPP

#include <functional>
#include "../Middleware/middleware_interfaces.hpp"
#include "isr_interfaces.hpp"


#define DBUS_BUFFER_LEN 18

namespace isr {
    namespace uart {
        enum class ECallbacks { RECEIVE_COMPLETE, ON_ERROR };

        using TISRState = MW_UART::Peripheral;
        using TInitFunc = std::function<bool(MW_UART::IUART&)>;
        using TISRRoutine =
            std::function<void(MW_UART::IUART&, MW_UART::Peripheral)>;

        class UART_ISR
            : public ISR<ECallbacks, TISRRoutine, TInitFunc, TISRState> {
           private:
            MW_UART::IUART& uart;

           public:
            UART_ISR(MW_UART::IUART& uart);

            [[nodiscard]] bool init() override;
            void run_isr_routines(ECallbacks callback_running,
                                  TISRState callback_state) override;
        };
    }  // namespace uart
}  // namespace isr

#endif