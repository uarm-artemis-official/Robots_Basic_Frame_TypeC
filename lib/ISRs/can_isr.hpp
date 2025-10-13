#ifndef __CAN_ISR_HPP
#define __CAN_ISR_HPP

#include "can.h"
#include "isr_interfaces.hpp"
#include "message_center.hpp"
#include "middleware_classes.hpp"
#include "subsystems_interfaces.hpp"
#include "subsystems_types.hpp"
#include "topics.hpp"

// TODO: Add middleware layer instead of inclusion of can.h.
namespace CAN_ISR {
    enum class Config { NORMAL, SENTRY_CHASSIS, SENTRY_GIMBAL };

    class CAN_ISR {
       private:
        mc2::MotorRead motor_read {};
        mc2::RobotMC& mc;
        Config config;

       public:
        CAN_ISR(mc2::RobotMC& mc_ref) : mc(mc_ref) {}
        void init(Config config);
        uint8_t get_free_buffer(uint32_t stdId);
        void read_motor_data(CAN_HandleTypeDef* hcan,
                             CAN_RxHeaderTypeDef& rx_header);
        void on_message_pending(CAN_HandleTypeDef* hcan);
    };
}  // namespace CAN_ISR

namespace isr {
    namespace can {
        enum class ECallbacks { MESSAGE_PENDING };

        using TISRState = MW_CAN::BUS;
        using TISRRoutine = void (*)(MW_CAN::ICAN&, TISRState);
        using TInitFunc = bool (*)(MW_CAN::ICAN&);

        class CAN_ISR
            : public ISR<ECallbacks, TISRRoutine, TInitFunc, TISRState> {
           private:
            MW_CAN::ICAN& can;

           public:
            CAN_ISR(MW_CAN::ICAN& can_ref);

            [[nodiscard]] bool init() override;

            void run_isr_routines(ECallbacks callback_running,
                                  TISRState callback_state) override;
        };
    }  // namespace can
}  // namespace isr

#endif