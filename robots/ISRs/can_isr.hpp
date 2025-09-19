#ifndef __CAN_ISR_HPP
#define __CAN_ISR_HPP

#include "can.h"
#include "subsystems_interfaces.hpp"
#include "subsystems_modules.hpp"
#include "subsystems_types.hpp"

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

#endif