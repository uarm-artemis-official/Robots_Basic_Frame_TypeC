#include "can_isr.hpp"
#include <algorithm>
#include "uarm_lib.hpp"

namespace isr {
    namespace can {
        CAN_ISR::CAN_ISR(MW_CAN::ICAN& can_ref) : can(can_ref) {}

        bool CAN_ISR::init() {
            return true;
        }

        void CAN_ISR::run_isr_routines(ECallbacks callback_running,
                                       TISRState callback_state) {
            switch (callback_running) {
                case ECallbacks::MESSAGE_PENDING: {
                    CANFrame frame;
                    can.receive_data(callback_state, MW_CAN::FIFO::FIFO_0,
                                     frame.stdid, frame.extid, frame.payload,
                                     frame.payload_length);
                    for (size_t i = 0; i < routines_size; i++) {
                        if (routines[i].second == callback_running) {
                            routines[i].first(callback_state, frame);
                        }
                    }
                    break;
                }
                default:
                    ASSERT(false, "Unhandled CAN ISR callback.");
                    break;
            }
        }

        bool CAN_ISR::on_register_init(size_t init_func_idx) {
            return init_funcs[init_func_idx](can);
        }

        bool CAN_ISR::on_register_routine(size_t routine_func_idx) {
            (void) routine_func_idx;
            return true;
        }
    }  // namespace can
}  // namespace isr