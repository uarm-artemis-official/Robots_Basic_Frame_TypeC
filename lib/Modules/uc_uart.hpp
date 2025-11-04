#ifndef __UC_UART_HPP
#define __UC_UART_HPP

#include <cstddef>
#include <cstdint>
#include "uarm_lib.hpp"

namespace uc_uart {
    constexpr uint8_t PROTOCOL_MAGIC = 0x7;
    constexpr size_t HEADER_SIZE = 4;
    constexpr size_t TRAILER_SIZE = 2;
    constexpr size_t MAX_CONTROL_FLOW_SIZE = 20;

    enum class FrameType : uint8_t {
        Data = 0x1,
        ControlFlow = 0x2,
        Invalid = 0xF
    };

    /*
     * Control Flow Frame Format:
     * +--------+--------+--------+--------+----------------+--------+--------+
     * | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4..N-3   | Byte N-2..N-1 |
     * +--------+--------+--------+--------+----------------+--------+--------+
     * | Magic  | Src/   | Msg    | Flow   | Parameters     | CRC16         |
     * | & Type | Dst    | Length | ID     | (optional)     |               |
     * +--------+--------+--------+--------+----------------+--------+--------+
     * 
     * Byte 0: Upper 4 bits = Protocol Magic (0x7)
     *         Lower 4 bits = Frame Type (0x2 for ControlFlow)
     * Byte 1: Upper 4 bits = Source address (0-15)
     *         Lower 4 bits = Destination address (0-15)
     * Byte 2: Message Length (including Flow ID and parameters)
     * Byte 3: Control Flow ID (0x1 for Ping, 0x2 for Pong)
     * Bytes 4..N-3: Optional parameters (empty for Ping/Pong)
     * Bytes N-2..N-1: CRC16 of all preceding bytes
     */
    enum class ControlFlowID : uint8_t { Ping = 0x1, Pong = 0x2 };

    inline uint8_t create_source_destination_byte(uint8_t source,
                                                  uint8_t destination) {
        ASSERT(source <= 0x0F, "Source address out of range (0-15)");
        ASSERT(destination <= 0x0F, "Destination address out of range (0-15)");
        return ((source & 0x0F) << 4) | (destination & 0x0F);
    }

    inline void parse_source_destination_byte(uint8_t byte, uint8_t& source,
                                              uint8_t& destination) {
        source = (byte >> 4) & 0x0F;
        destination = byte & 0x0F;
    }

    template <size_t MAX_MESSAGE_SIZE>
    class UC_UART {
       private:
        uint8_t source_address;
        uint8_t
            send_payload_buffer[MAX_MESSAGE_SIZE + HEADER_SIZE + TRAILER_SIZE];

       public:
        UC_UART(uint8_t _source_address) : source_address(_source_address) {};

        [[nodiscard]] bool set_send_data(uint8_t destination,
                                         uint8_t message_id,
                                         const uint8_t* payload, size_t length);

        [[nodiscard]] bool get_control_flow_frame(uint8_t* frame_destination,
                                                  uint8_t destination,
                                                  ControlFlowID flow_id,
                                                  void* params);

        [[nodiscard]] bool process_control_flow_frame(const uint8_t* frame,
                                                      size_t frame_length,
                                                      void* message_received);

        [[nodiscard]] bool process_received_data(
            const uint8_t* frame, size_t frame_length, uint8_t& message_id,
            uint8_t* payload_buffer, size_t* payload_length = nullptr);

        FrameType get_frame_type(const uint8_t* frame, size_t frame_length);
    };
}  // namespace uc_uart

#ifndef __UC_UART_IPP
#include "uc_uart.ipp"
#endif

#endif