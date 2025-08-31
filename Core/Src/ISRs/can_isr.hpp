#ifndef __CAN_ISR_HPP
#define __CAN_ISR_HPP

#include "can.h"
#include "subsystems_interfaces.hpp"
#include "subsystems_types.hpp"

namespace CAN_ISR {
    enum class Config { NORMAL, SENTRY_CHASSIS, SENTRY_GIMBAL };

    class CAN_ISR {
       private:
        MotorReadMessage_t read_message;
        IMessageCenter& message_center;
        Config config;

       public:
        CAN_ISR(IMessageCenter& _message_center)
            : message_center(_message_center) {}
        void init(Config config);
        uint8_t get_free_buffer(uint32_t stdId);
        void read_motor_data(CAN_HandleTypeDef* hcan,
                             CAN_RxHeaderTypeDef& rx_header);
        void message_pending(CAN_HandleTypeDef* hcan);
    };
}  // namespace CAN_ISR

#endif