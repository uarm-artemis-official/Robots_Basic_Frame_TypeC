#ifndef __CAN_ISR_HPP
#define __CAN_ISR_HPP

#include <functional>
#include "isr_interfaces.hpp"
#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

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
            explicit CAN_ISR(MW_CAN::ICAN& can_ref) : can(can_ref) {}

            [[nodiscard]] bool init() override {
                initialized = true;
                return true;
            }

            [[nodiscard]] bool on_register_init(size_t init_func_idx) override {
                return init_funcs[init_func_idx](can);
            }

            [[nodiscard]] bool on_register_routine(
                size_t routine_func_idx) override {
                (void) routine_func_idx;
                return true;
            }

            void run_isr_routines(ECallbacks callback_running,
                                  TISRState callback_state) override {
                switch (callback_running) {
                    case ECallbacks::MESSAGE_PENDING: {
                        CANFrame frame;
                        bool received_new_message = can.receive_data(
                            callback_state, MW_CAN::FIFO::FIFO_0, frame.stdid,
                            frame.extid, frame.payload, frame.payload_length);

                        if (received_new_message) {
                            for (size_t i = 0; i < routines_size; i++) {
                                if (routines[i].second == callback_running) {
                                    routines[i].first(callback_state, frame);
                                }
                            }
                        }
                        break;
                    }
                    default:
                        ASSERT(false, "Unhandled CAN ISR callback.");
                        break;
                }
            }
        };

        bool install_isr(CAN_ISR* can_isr_ref);
    }  // namespace can
}  // namespace isr

#endif