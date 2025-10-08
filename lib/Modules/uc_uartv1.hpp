#ifndef __UC_UARTV1_HPP
#define __UC_UARTV1_HPP

#include <cstdint>
#include "middleware_interfaces.hpp"
#include "uc_uart_interface.hpp"

namespace uc_uart {
    namespace v1 {
        constexpr uint8_t SOF_MAGIC = 0x7;
        constexpr uint8_t PROTOCOL_VERSION = 0x1;
        constexpr uint8_t FIRST_BYTE = (SOF_MAGIC << 4) & PROTOCOL_VERSION;

        enum class ControlFlowID { Ping, Pong };
        enum class Config { FailFast, Robust };
        enum class FrameType { Data = 0x1, ControlFlow = 0x2 };

        class UC_UARTV1 : UC_UART<UC_UARTV1, ControlFlowID> {
           private:
            Config config;
            MW_UART::IUART& uart;

           public:
            UC_UARTV1(Config _config, MW_UART::IUART& _uart);

            void send_data_impl(const uint16_t id, const uint8_t* data,
                                size_t length);

            void send_control_flow_impl(const uint16_t id,
                                        ControlFlowID control_id, void* body);

            bool process_receive_message_impl(void* dst, uint8_t* buffer,
                                              size_t length);

            // Calculate checksum for a message
            uint16_t calculate_checksum(const uint8_t* data, size_t length);

            // Verify integrity of a message using checksum
            bool verify_message_integrity(const uint8_t* data, size_t length);

            void check(bool cond, const char* msg);
        };
    }  // namespace v1
}  // namespace uc_uart

#endif