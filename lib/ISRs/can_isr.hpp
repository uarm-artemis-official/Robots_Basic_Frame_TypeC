#ifndef __CAN_ISR_HPP
#define __CAN_ISR_HPP

#include <functional>
#include "../Middleware/middleware_interfaces.hpp"
#include "isr_interfaces.hpp"


namespace isr {
    namespace can {
        // TODO: Incorporate this into CAN middleware.
        struct CANFrame {
            uint32_t stdid;
            uint32_t extid;
            uint32_t payload_length;
            uint8_t payload[8];
        };

        enum class ECallbacks { MESSAGE_PENDING };

        using TISRState = MW_CAN::BUS;
        using TISRRoutine = std::function<void(MW_CAN::BUS, CANFrame)>;
        using TInitFunc = std::function<bool(MW_CAN::ICAN&)>;

        class CAN_ISR
            : public ISR<ECallbacks, TISRRoutine, TInitFunc, TISRState> {
           private:
            MW_CAN::ICAN& can;

           public:
            CAN_ISR(MW_CAN::ICAN& can_ref);

            [[nodiscard]] bool init() override;
            [[nodiscard]] bool on_register_init(size_t init_func_idx) override;
            [[nodiscard]] bool on_register_routine(size_t routine_func_idx) override;
            void run_isr_routines(ECallbacks callback_running,
                                  TISRState callback_state) override;
        };
    }  // namespace can
}  // namespace isr

#endif