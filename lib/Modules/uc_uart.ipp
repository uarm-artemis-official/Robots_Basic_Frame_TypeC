#ifndef __UC_UART_IPP
#define __UC_UART_IPP

#include <cstring>
#include "uc_uart.hpp"

namespace uc_uart {
    // Implement a crc16 function or include your CRC library here
    // TODO: Move to algorithms file.
    inline uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]);
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    template <size_t MAX_MESSAGE_SIZE>
    [[nodiscard]] bool UC_UART<MAX_MESSAGE_SIZE>::set_send_data(
        uint8_t destination, uint8_t message_id, const uint8_t* payload,
        size_t length) {
        ASSERT(length <= MAX_MESSAGE_SIZE,
               "Payload length exceeds maximum message size.");
        ASSERT(payload != nullptr, "Payload pointer is null.");

        // Construct header
        send_payload_buffer[0] =
            PROTOCOL_MAGIC << 4 |
            static_cast<uint8_t>(comm::protocol::FrameType::Data);
        send_payload_buffer[1] =
            create_source_destination_byte(source_address, destination);
        send_payload_buffer[2] = message_id;
        send_payload_buffer[3] = static_cast<uint8_t>(length);

        // Copy payload
        memcpy(&send_payload_buffer[HEADER_SIZE], payload, length);

        // Compute and append trailer (simple checksum)
        uint16_t checksum = crc16(send_payload_buffer, HEADER_SIZE + length);
        memcpy(&send_payload_buffer[HEADER_SIZE + length], &checksum,
               sizeof(checksum));

        return true;
    }

    template <size_t MAX_MESSAGE_SIZE>
    bool UC_UART<MAX_MESSAGE_SIZE>::format_send_data(
        const uint8_t* payload, comm::protocol::TopicMessageMeta meta,
        uint8_t* dst_buffer) {
        ASSERT(payload != nullptr, "Payload pointer is null.");
        ASSERT(dst_buffer != nullptr, "Destination buffer pointer is null.");
        ASSERT(meta.payload_length <= MAX_MESSAGE_SIZE,
               "Payload length exceeds maximum message size.");
        // Construct header
        dst_buffer[0] = PROTOCOL_MAGIC << 4 |
                        static_cast<uint8_t>(comm::protocol::FrameType::Data);
        dst_buffer[1] =
            create_source_destination_byte(meta.source, meta.destination);
        dst_buffer[2] = meta.message_id;
        dst_buffer[3] = static_cast<uint8_t>(meta.payload_length);
        // Copy payload
        memcpy(&dst_buffer[HEADER_SIZE], payload, meta.payload_length);
        // Compute and append trailer (simple checksum)
        uint16_t checksum =
            crc16(dst_buffer, HEADER_SIZE + meta.payload_length);
        memcpy(&dst_buffer[HEADER_SIZE + meta.payload_length], &checksum,
               sizeof(checksum));
        return true;
    }

    template <size_t MAX_MESSAGE_SIZE>
    [[nodiscard]] bool UC_UART<MAX_MESSAGE_SIZE>::get_control_flow_frame(
        uint8_t* frame_destination, uint8_t destination, ControlFlowID flow_id,
        void* params) const {
        (void) params;  // Unused for now since ping/pong have no parameters
        ASSERT(flow_id == ControlFlowID::Ping || flow_id == ControlFlowID::Pong,
               "Unsupported ControlFlowID");

        memset(frame_destination, 0,
               HEADER_SIZE + TRAILER_SIZE);  // Clear frame buffer
        // Construct header
        frame_destination[0] =
            PROTOCOL_MAGIC << 4 |
            static_cast<uint8_t>(comm::protocol::FrameType::ControlFlow);
        frame_destination[1] =
            create_source_destination_byte(source_address, destination);
        frame_destination[2] = 0;  // Length is 1 byte (just the flow ID)
        frame_destination[3] = static_cast<uint8_t>(flow_id);
        // Compute and append trailer (simple checksum)
        uint16_t checksum = crc16(frame_destination, HEADER_SIZE);
        memcpy(&frame_destination[HEADER_SIZE], &checksum, sizeof(checksum));

        return true;
    }

    template <size_t MAX_MESSAGE_SIZE>
    [[nodiscard]] bool UC_UART<MAX_MESSAGE_SIZE>::process_control_flow_frame(
        const uint8_t* frame, size_t frame_length, void* message_received) {
        ASSERT(frame_length >= HEADER_SIZE + TRAILER_SIZE,
               "Frame length too short for control flow frame.");
        ASSERT(message_received != nullptr,
               "message_received pointer is null.");

        // Verify checksum
        uint16_t received_checksum;
        memcpy(&received_checksum, &frame[frame_length - TRAILER_SIZE],
               sizeof(received_checksum));
        uint16_t computed_checksum = crc16(frame, frame_length - TRAILER_SIZE);
        if (received_checksum != computed_checksum) {
            return false;  // Invalid checksum: discard frame.
        }

        size_t params_length = static_cast<size_t>(frame[2]);
        ControlFlowID flow_id = static_cast<ControlFlowID>(frame[3]);
        switch (flow_id) {
            case ControlFlowID::Ping:
            case ControlFlowID::Pong:
                if (params_length == 0) {
                    *static_cast<ControlFlowID*>(message_received) = flow_id;
                    return true;
                } else {
                    return false;
                }
            default:
                ASSERT(false, "Unsupported ControlFlowID received.");
                return false;
        }
    }

    template <size_t MAX_MESSAGE_SIZE>
    [[nodiscard]] bool UC_UART<MAX_MESSAGE_SIZE>::process_received_data(
        const uint8_t* frame, size_t frame_length, uint8_t& message_id,
        uint8_t* payload_buffer, size_t* payload_length) {
        ASSERT(frame_length >= HEADER_SIZE + TRAILER_SIZE,
               "Frame length too short for data frame.");
        ASSERT(payload_buffer != nullptr, "Payload buffer pointer is null.");

        // Verify checksum
        uint16_t received_checksum;
        memcpy(&received_checksum, &frame[frame_length - TRAILER_SIZE],
               sizeof(received_checksum));
        uint16_t computed_checksum = crc16(frame, frame_length - TRAILER_SIZE);
        if (received_checksum != computed_checksum) {
            return false;  // Invalid checksum: discard frame.
        }

        message_id = frame[3];
        size_t data_length = static_cast<size_t>(frame[2]);

        if (data_length > MAX_MESSAGE_SIZE || frame_length != data_length) {
            return false;
        }

        // Copy payload
        memcpy(payload_buffer, &frame[HEADER_SIZE], data_length);
        if (payload_length != nullptr) {
            *payload_length = data_length;
        }

        return true;
    }

    template <size_t MAX_MESSAGE_SIZE>
    comm::protocol::FrameType UC_UART<MAX_MESSAGE_SIZE>::get_frame_type(
        const uint8_t* frame, size_t frame_length) {
        if (frame_length < HEADER_SIZE + TRAILER_SIZE) {
            return comm::protocol::FrameType::Invalid;
        }

        uint8_t frame_id = frame[0] & 0x0F;
        switch (frame_id) {
            case static_cast<uint8_t>(comm::protocol::FrameType::Data):
                [[fallthrough]];
            case static_cast<uint8_t>(comm::protocol::FrameType::ControlFlow):
                return static_cast<comm::protocol::FrameType>(frame_id);
            default:
                return comm::protocol::FrameType::Invalid;
        }
    }

    template <size_t MAX_MESSAGE_SIZE>
    comm::protocol::TopicMessageMeta
    UC_UART<MAX_MESSAGE_SIZE>::parse_meta_from_headers(
        const uint8_t* header) const {
        comm::protocol::TopicMessageMeta meta;
        parse_source_destination(header, meta.source, meta.destination);
        parse_message_id(header, meta.message_id);
        parse_length(header, meta.payload_length);
        return meta;
    }

};  // namespace uc_uart

#endif