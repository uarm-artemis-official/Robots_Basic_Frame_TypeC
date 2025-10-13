#ifndef __UART_ISR_HPP
#define __UART_ISR_HPP

#include "subsystems_defines.hpp"
#include "subsystems_interfaces.hpp"
#include "topics.hpp"
#include "usart.h"

#define DBUS_BUFFER_LEN 18

// TODO: Add middleware layer instead of inclusion of usart.h.
namespace UART_ISR {
    enum class Config {
        CHASSIS,
        GIMBAL,
        AUTO_AIM,
        NONE,
    };

    struct UARTISRState {
        uint32_t complete_count;
        uint32_t error_count;
        mc2::UCPackIn uc_pack_in;
        mc2::RefereeIn referee_in;
        mc2::RCRaw rc_raw;
    };

    class UART_ISR {
       private:
        Config config;
        UARTISRState state;
        mc2::RobotMC& mc;

       public:
        UART_ISR(mc2::RobotMC& mc_ref);
        void init(Config _config);
        void on_receive_complete(UART_HandleTypeDef* huart);
        void on_error(UART_HandleTypeDef* huart);
    };
}  // namespace UART_ISR

#endif