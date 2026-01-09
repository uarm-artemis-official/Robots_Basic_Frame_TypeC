#ifndef __DEBUG_HPP
#define __DEBUG_HPP

#include <cstddef>
#include <span>
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"

namespace modules {
    namespace debug {
        enum class BoardConfig { CHASSIS, GIMBAL, UNKNOWN };

        class Debug {
           private:
            MW_GPIO::IGPIO& gpio;
            MW_UART::IUART& uart;
            MW_UART::Peripheral uart_debug_peripheral;

           public:
            Debug(MW_GPIO::IGPIO& gpio, MW_UART::IUART& uart)
                : gpio(gpio), uart(uart) {}

            bool init() {
                switch (get_board_config()) {
                    case BoardConfig::CHASSIS:
                        uart_debug_peripheral = MW_UART::Peripheral::UART1;
                        break;
                    case BoardConfig::GIMBAL:
                        uart_debug_peripheral = MW_UART::Peripheral::UART6;
                        break;
                    default:
                        ASSERT(false, "Unknown board configuration.");
                        return false;
                }
                return true;
            }

            BoardConfig get_board_config(void) {
                MW_GPIO::State pin_state =
                    gpio.read_pin(MW_GPIO::Port::PORT_F, MW_GPIO::Pin::PIN_1);
                if (pin_state == MW_GPIO::State::HIGH) {
                    return BoardConfig::GIMBAL;
                } else {
                    return BoardConfig::CHASSIS;
                }
            }

            void send_debug_message(std::span<const std::byte> data,
                                    uint32_t timeout = 1) {
                uart.send_data(uart_debug_peripheral, data, timeout);
            }
        };
    }  // namespace debug
}  // namespace modules

#endif