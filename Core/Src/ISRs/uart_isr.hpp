#ifndef __UART_ISR_HPP
#define __UART_ISR_HPP

#include "subsystems_defines.hpp"
#include "subsystems_interfaces.hpp"

#define DBUS_BUFFER_LEN 18

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
        uint8_t rc_frame_buffer[DBUS_BUFFER_LEN];
        uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE];
        uint8_t ref_rx_frame[MAX_REF_BUFFER_SIZE];
    };

    class UART_ISR {
       private:
        Config config;
        UARTISRState state;
        IMessageCenter& message_center;

       public:
        UART_ISR(IMessageCenter& _message_center);
        void init(Config _config);
        void on_receive_complete(UART_HandleTypeDef* huart);
        void on_error(UART_HandleTypeDef* huart);
    };
}  // namespace UART_ISR

#endif