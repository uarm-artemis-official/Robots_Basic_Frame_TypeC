#ifndef __DEBUG_HPP
#define __DEBUG_HPP

#include <cstddef>
#include <optional>
#include <source_location>
#include <span>
#include <string>
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "uarm_lib.hpp"

namespace modules {
    namespace debug {
        using UARTAccessToken = uint32_t;

        enum class BoardConfig { CHASSIS, GIMBAL, UNKNOWN };

        class Debug {
           private:
            MW_GPIO::IGPIO& gpio;
            MW_UART::IUART& uart;
            MW_UART::Peripheral uart_debug_peripheral =
                MW_UART::Peripheral::Unknown;

            UARTAccessToken current_token = 0;
            bool is_initialized = false;

           public:
            Debug(MW_GPIO::IGPIO& gpio, MW_UART::IUART& uart)
                : gpio(gpio), uart(uart) {}

            bool init() {
                switch (get_board_config()) {
                    case BoardConfig::CHASSIS:
                        uart_debug_peripheral = MW_UART::Peripheral::UART_1;
                        break;
                    case BoardConfig::GIMBAL:
                        uart_debug_peripheral = MW_UART::Peripheral::UART_6;
                        break;
                    default:
                        ASSERT(false, "Unknown board configuration.");
                        return false;
                }
                is_initialized = true;
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

            [[nodiscard]] std::optional<UARTAccessToken> reserve_debug_uart(
                std::source_location location =
                    std::source_location::current()) {
                if (current_token != 0) {
                    return std::nullopt;
                }

                const uint64_t modulus = 1000000007;
                uint64_t new_token = 0;

                std::string file = location.file_name();
                for (char c : file) {
                    new_token += static_cast<uint64_t>(c);
                }
                new_token = (((new_token % modulus) * (location.line() + 1)) %
                             modulus * (location.column() + 1)) %
                            modulus;

                ASSERT(new_token != 0,
                       "Generated UART access token cannot be zero.");
                current_token = new_token;

                return current_token;
            }

            bool release_debug_uart(UARTAccessToken token) {
                if (token == current_token) {
                    current_token = 0;
                    return true;
                }
                return false;
            }

            void send_debug_message(UARTAccessToken token,
                                    std::span<const std::byte> data,
                                    uint32_t timeout = 1) {
                ASSERT(is_initialized, "Debug module not initialized.");
                if (token == current_token) {
                    uart.send_data(uart_debug_peripheral, data, timeout);
                }
            }
        };
    }  // namespace debug
}  // namespace modules

#endif