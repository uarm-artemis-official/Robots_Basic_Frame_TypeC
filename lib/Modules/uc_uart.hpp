#ifndef __UC_UART_HPP
#define __UC_UART_HPP

#include <cstddef>
#include <cstdint>
#include "comm_protocol_interface.hpp"
#include "uarm_lib.hpp"

namespace uc_uart {
    constexpr uint8_t PROTOCOL_MAGIC = 0x7;
    constexpr size_t HEADER_SIZE = 4;
    constexpr size_t TRAILER_SIZE = 2;
    constexpr size_t MAX_CONTROL_FLOW_SIZE = 20;

    /*
     * Data Frame Format:
     * +--------+--------+--------+--------+----------------+--------+--------+
     * | Byte 0 | Byte 1 | Byte 2 | Byte 3 | Byte 4..N-3    | Byte N-2..N-1 |
     * +--------+--------+--------+--------+----------------+--------+--------+
     * | Magic  | Src/   | Msg ID | Length | Payload        | CRC16         |
     * | & Type | Dst    |        |        | (variable)     |               |
     * +--------+--------+--------+--------+----------------+--------+--------+
     *
     * Description:
     * - Header (HEADER_SIZE == 4 bytes)
     *   Byte 0: Upper 4 bits = Protocol Magic (0x7)
     *           Lower 4 bits = Frame Type (0x1 for Data)
     *   Byte 1: Upper 4 bits = Source address (0-15)
     *           Lower 4 bits = Destination address (0-15)
     *   Byte 2: Message identifier (message center topic ID)
     *   Byte 3: Payload length in bytes (not including header or trailer)
     *
     * - Payload (Byte 4 .. Byte 4 + Length - 1)
     *   Contains the message center topic data bytes. Maximum payload length is
     *   determined by surrounding code and the available buffer space
     *   (consider HEADER_SIZE + payload + TRAILER_SIZE constraints).
     *
     * - Trailer (TRAILER_SIZE == 2 bytes)
     *   Bytes N-2..N-1: CRC16 over all preceding bytes (header + payload).
     *
     * Notes:
     * - Length is limited to fit in a single byte. Implementations should
     *   validate length against MAX_MESSAGE_SIZE and available buffer space.
     * - CRC16 covers header and payload; receivers must verify CRC before
     *   accepting the payload.
     */
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

    inline void parse_source_destination(const uint8_t* header, uint8_t& source,
                                         uint8_t& destination) {
        source = (header[1] >> 4) & 0x0F;
        destination = header[1] & 0x0F;
    }

    inline void parse_message_id(const uint8_t* header, uint8_t& message_id) {
        message_id = header[2];
    }

    inline void parse_length(const uint8_t* header, size_t& length) {
        length = header[3];
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

        /**
         * @brief Formats a payload and its metadata into a UC_UART data frame.
         * 
         * @param payload Pointer to the message payload data.
         * @pre payload pointer is not nullptr.
         * @param meta Metadata associated with the message (source, destination, length, message ID).
         * @param dst_buffer Pointer to the destination buffer to hold the formatted data frame.
         * @pre dst_buffer pointer is not nullptr.
         * @return true if formatting was successful, otherwise false.
         * @note The current implementation of this method returns true or errors, it does not return false.
         */
        [[nodiscard]] bool format_send_data(
            const uint8_t* payload, comm::protocol::TopicMessageMeta meta,
            uint8_t* dst_buffer);

        /**
         * @brief Creates a control flow frame.
         * 
         * @param frame_destination Pointer to the destination buffer to hold the created control flow frame.
         * @pre frame_destination pointer is not nullptr.
         * @param destination Identifier for the message destination.
         * @pre destination must be a valid node ID (1..15).
         * @param flow_id The type of control flow frame to create.
         * @param params Pointer to additional parameters for frame creation.
         * @note Currently there are no additional parameters used for any control flow frames.
         */
        [[nodiscard]] bool get_control_flow_frame(uint8_t* frame_destination,
                                                  uint8_t destination,
                                                  ControlFlowID flow_id,
                                                  void* params) const;

        [[nodiscard]] bool process_control_flow_frame(const uint8_t* frame,
                                                      size_t frame_length,
                                                      void* message_received);

        [[nodiscard]] bool process_received_data(
            const uint8_t* frame, size_t frame_length, uint8_t& message_id,
            uint8_t* payload_buffer, size_t* payload_length = nullptr);

        comm::protocol::TopicMessageMeta parse_meta_from_headers(
            const uint8_t* header) const;

        comm::protocol::FrameType get_frame_type(const uint8_t* frame,
                                                 size_t frame_length);
    };
}  // namespace uc_uart

#ifndef __UC_UART_IPP
#include "uc_uart.ipp"
#endif

#endif