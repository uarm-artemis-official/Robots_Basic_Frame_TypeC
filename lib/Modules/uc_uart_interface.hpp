#ifndef __UC_UART_HPP
#define __UC_UART_HPP

#include <cstddef>
#include <cstdint>

namespace uc_uart {
    /**
     * @brief UART communication protocol class for microcontroller to upper computer.
     * 
     * Provides basic send/receive and message integrity checking.
     */
    template <typename TProtocol, typename TControlFlowID>
    class UC_UART {
       public:
        // Send a message over UART
        void send_data(const uint16_t id, const uint8_t* data, size_t length) {
            TProtocol* protocol = static_cast<TProtocol*>(this);
            protocol.send_data_impl(data, length);
        }

        void send_control_flow(const uint16_t id, TControlFlowID control_id,
                               void* body) {
            TProtocol* protocol = static_cast<TProtocol*>(this);
            protocol.send_control_flow_impl(id, control_id, body);
        }

        // Receive a message from UART
        bool process_receive_message(void* dst, uint8_t* buffer,
                                     size_t length) {
            TProtocol* protocol = static_cast<TProtocol*>(this);
            return protocol.process_receive_message_impl(dst, buffer, length);
        }
    };

}  // namespace uc_uart

#endif