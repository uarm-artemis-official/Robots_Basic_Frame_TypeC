#ifndef __UC_UARTV1_HPP
#define __UC_UARTV1_HPP

#include <cstdint>
#include "middleware_interfaces.hpp"
#include "uc_uart_interface.hpp"

namespace uc_uart {
    namespace v1 {
        /**
         * UART Protocol Frame Format
         *
         * General Data Frame Format:
         *  +-----+--------+-----------+-----------+--------------+---------+-------------------+
         *  | SOF |Version | Identifier| Frame type| Payload len  | Payload | CRC16 checksum    |
         *  |(4b) | (4b)   | (8b)      | (4b)      | (4b)         |(0-8B)   | (16b)             |
         *  +-----+--------+-----------+-----------+--------------+---------+-------------------+
         *  |0x7  | 0x1    |    *      | 0x1       |    *         |   *     |   *               |
         *  +-----+--------+-----------+-----------+--------------+---------+-------------------+
         * 
         * General Control Flow Frame Format:
         *  +-----+--------+-----------+-----------+---------------------+-------------------+
         *  | SOF |Version | Identifier| Frame type| Control flow msg ID | CRC16 checksum    |
         *  |(4b) | (4b)   | (8b)      | (4b)      | (4b)                | (16b)             |
         *  +-----+--------+-----------+-----------+---------------------+-------------------+
         *  |0x7  | 0x1    |    *      | 0x2       |    *                |   *               |
         *  +-----+--------+-----------+-----------+---------------------+-------------------+
         *
         * Example Data Frame:
         *  [SOF][Version][Identifier][FrameType][PayloadLen][Payload...][CRC16]
         *
         * Example Control Flow Frame:
         *  [SOF][Version][Identifier][FrameType][ControlFlowID][CRC16]
         * 
         * Frame fields are little-endian.
         * 
         * The upper nibble comes first before the lower nibble (e.g. SOF is the upper nibble and 
         * version is the lower nibble of 0th byte). This ordering follows for all other nibbles.
         */

        constexpr uint8_t SOF_MAGIC = 0x7;
        constexpr uint8_t PROTOCOL_VERSION = 0x1;
        constexpr uint8_t FIRST_BYTE = (SOF_MAGIC << 4) | PROTOCOL_VERSION;
        constexpr size_t MAX_DATA_MESSAGE_LENGTH = 13;
        constexpr size_t MAX_CONTROL_FLOW_MESSAGE_LENGTH = 5;

        enum class ControlFlowID { Ping, Pong };
        enum class Config { FailFast, Robust };
        enum class FrameType { Data = 0x1, ControlFlow = 0x2 };

        // Metadata helpers for buffer access
        // All helper functions assume that the buffer is sufficiently large.
        constexpr FrameType get_frame_type(const uint8_t* buffer) {
            return static_cast<FrameType>(buffer[2] >> 4);
        }

        constexpr uint8_t get_identifier(const uint8_t* buffer) {
            return buffer[1];
        }

        constexpr uint8_t get_payload_length(const uint8_t* buffer) {
            return buffer[2] & 0x0F;
        }

        constexpr uint8_t get_control_flow_id(const uint8_t* buffer) {
            return buffer[2] & 0x0F;
        }

        // Setters for metadata fields
        inline void set_frame_type(uint8_t* buffer, FrameType type) {
            buffer[2] = (static_cast<uint8_t>(type) << 4) | (buffer[2] & 0x0F);
        }

        inline void set_identifier(uint8_t* buffer, uint8_t id) {
            buffer[1] = id;
        }

        inline void set_payload_length(uint8_t* buffer,
                                       uint8_t payload_length) {
            buffer[2] = (payload_length & 0x0F) | (buffer[2] & 0xF0);
        }

        inline void set_control_flow_id(uint8_t* buffer,
                                        uint8_t control_flow_id) {
            buffer[2] = (control_flow_id & 0x0F) | (buffer[2] & 0xF0);
        }

        class UC_UARTV1 : UC_UART<UC_UARTV1, ControlFlowID> {
           private:
            Config config;
            MW_UART::IUART& uart;
            MW_UART::Peripheral peripheral;

           public:
            UC_UARTV1(Config _config, MW_UART::IUART& _uart,
                      MW_UART::Peripheral _peripheral);

            void send_data_impl(const uint8_t id, const uint8_t* data,
                                size_t length);

            void send_control_flow_impl(const uint8_t id,
                                        ControlFlowID control_id, void* body,
                                        size_t length);

            bool process_receive_message_impl(void* dst, uint8_t* buffer,
                                              size_t length);

            bool is_start_of_frame_impl(uint8_t* buffer, size_t length);

            // Calculate checksum for a message
            uint16_t calculate_checksum(const uint8_t* data, size_t length);

            // Verify integrity of a message using checksum
            bool verify_message_integrity(const uint8_t* data, size_t length);

            void check(bool cond, const char* msg);
        };
    }  // namespace v1
}  // namespace uc_uart

#endif