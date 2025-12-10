#ifndef __CAN2_TP_IPP
#define __CAN2_TP_IPP

#include <algorithm>
#include <cstring>
#include "../uarm_lib.hpp"
#include "can2_tp.hpp"

namespace comm::can2_tp {
    inline namespace v1 {
        // constexpr helpers for readable mask generation
        constexpr uint32_t bit_mask(unsigned bits) noexcept {
            return (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
        }

        constexpr uint32_t field_mask(unsigned shift, unsigned bits) noexcept {
            return bit_mask(bits) << shift;
        }

        // Bit layout (from LSB -> MSB) for stdid:
        // [ frame_type: 3 bits ] [ source: 4 bits ] [ destination: 4 bits ]
        constexpr unsigned FRAME_SHIFT = 0;
        constexpr unsigned FRAME_BITS = 3;
        constexpr uint32_t FRAME_MASK = field_mask(FRAME_SHIFT, FRAME_BITS);

        constexpr unsigned SRC_SHIFT = FRAME_SHIFT + FRAME_BITS;  // 3
        constexpr unsigned SRC_BITS = 4;
        constexpr uint32_t SRC_MASK = field_mask(SRC_SHIFT, SRC_BITS);

        constexpr unsigned DEST_SHIFT = SRC_SHIFT + SRC_BITS;  // 7
        constexpr unsigned DEST_BITS = 4;
        constexpr uint32_t DEST_MASK = field_mask(DEST_SHIFT, DEST_BITS);

        // Extended ID layout (from LSB -> MSB):
        // [ length/index: 8 bits ] [ message_id: 8 bits ] [ reserved: 2 bits ] ...
        constexpr unsigned LENGTH_SHIFT = 0;
        constexpr unsigned LENGTH_BITS = 8;
        constexpr uint32_t LENGTH_MASK = field_mask(LENGTH_SHIFT, LENGTH_BITS);

        constexpr unsigned MSGID_SHIFT = LENGTH_SHIFT + LENGTH_BITS;  // 8
        constexpr unsigned MSGID_BITS = 8;
        constexpr uint32_t MSGID_MASK = field_mask(MSGID_SHIFT, MSGID_BITS);

        inline void set_frame_type(comm::protocol::FrameType frame_type,
                                   uint32_t& stdid) {
            stdid &= ~FRAME_MASK;
            stdid |= ((static_cast<uint32_t>(frame_type) & bit_mask(FRAME_BITS))
                      << FRAME_SHIFT);
        }

        inline comm::protocol::FrameType get_frame_type(uint32_t stdid) {
            uint32_t v = (stdid & FRAME_MASK) >> FRAME_SHIFT;
            return static_cast<comm::protocol::FrameType>(v);
        }

        inline void set_destination(uint8_t destination, uint32_t& stdid) {
            if (destination == 0 || destination > bit_mask(DEST_BITS))
                return;
            stdid &= ~DEST_MASK;
            stdid |= (static_cast<uint32_t>(destination) & bit_mask(DEST_BITS))
                     << DEST_SHIFT;
        }

        inline uint8_t get_destination(uint32_t stdid) {
            return static_cast<uint8_t>((stdid & DEST_MASK) >> DEST_SHIFT);
        }

        inline void set_source(uint8_t source, uint32_t& stdid) {
            if (source == 0 || source > bit_mask(SRC_BITS))
                return;
            stdid &= ~SRC_MASK;
            stdid |= (static_cast<uint32_t>(source) & bit_mask(SRC_BITS))
                     << SRC_SHIFT;
        }

        inline uint8_t get_source(uint32_t stdid) {
            return static_cast<uint8_t>((stdid & SRC_MASK) >> SRC_SHIFT);
        }

        inline void set_index(uint8_t index, uint32_t& extid) {
            // write index into bits [7:0] of the provided id (treated as extid)
            extid &= ~LENGTH_MASK;
            extid |= (static_cast<uint32_t>(index) & bit_mask(LENGTH_BITS))
                     << LENGTH_SHIFT;
        }

        inline uint8_t get_index(uint32_t extid) {
            // read low 8 bits (index/length field)
            return static_cast<uint8_t>((extid & LENGTH_MASK) >> LENGTH_SHIFT);
        }

        inline void set_length(size_t length, uint32_t& extid) {
            uint32_t val =
                static_cast<uint32_t>(length) & bit_mask(LENGTH_BITS);
            extid &= ~LENGTH_MASK;
            extid |= (val << LENGTH_SHIFT);
        }

        // Length is additionally used for storing control flow IDs in some control flow frames
        // (Ping and Pong).
        inline size_t get_length(uint32_t extid) {
            return static_cast<size_t>((extid & LENGTH_MASK) >> LENGTH_SHIFT);
        }

        inline void set_message_id(uint8_t message_id, uint32_t& extid) {
            extid &= ~MSGID_MASK;
            extid |= (static_cast<uint32_t>(message_id) & bit_mask(MSGID_BITS))
                     << MSGID_SHIFT;
        }

        inline uint8_t get_message_id(uint32_t extid) {
            return static_cast<uint8_t>((extid & MSGID_MASK) >> MSGID_SHIFT);
        }

        // Template method implementations
        template <size_t MaxMessageSize>
        void CAN2TP<MaxMessageSize>::set_send_message(const uint8_t* message,
                                                      size_t length,
                                                      uint8_t message_id,
                                                      uint8_t destination) {
            ASSERT(length > MaxMessageSize,
                   "CAN2TP message length exceeds buffer capacity");
            ASSERT(length < MIN_SEGMENT_MESSAGE_LENGTH,
                   "Message length must be greater than "
                   "MIN_SEGMENT_MESSAGE_LENGTH for segmentation");

            // initialize send buffer struct fields
            send_buffer.used_send_buffer_length = 0;

            // update meta
            send_buffer.meta.source = source;
            send_buffer.meta.destination = destination;
            send_buffer.meta.payload_length = length;
            send_buffer.meta.message_id = message_id;

            memcpy(send_buffer.send_message_buffer, message, length);
        }

        template <size_t MaxMessageSize>
        bool CAN2TP<MaxMessageSize>::is_sending_message() {
            return send_buffer.used_send_buffer_length ==
                       send_buffer.payload_length &&
                   send_buffer.payload_length != 0;
        }

        template <size_t MaxMessageSize>
        [[nodiscard]] bool CAN2TP<MaxMessageSize>::get_next_send_fragment(
            CAN2BFrame& frame) {
            // If nothing to send
            if (send_buffer.used_send_buffer_length >=
                send_buffer.meta.payload_length) {
                return false;
            }

            const size_t remaining = send_buffer.meta.payload_length -
                                     send_buffer.used_send_buffer_length;
            const size_t max_payload = sizeof(frame.payload);  // 8
            const size_t chunk =
                (remaining < max_payload) ? remaining : max_payload;

            // Clear identifiers
            frame.stdid = 0;
            frame.extid = 0;

            if (send_buffer.used_send_buffer_length == 0) {
                // First Frame
                set_frame_type(comm::protocol::FrameType::FirstFrame,
                               frame.stdid);
                // extid low byte for first frame contains the total length per API note
                set_length(send_buffer.meta.payload_length, frame.extid);
            } else {
                // Consecutive Frame
                set_frame_type(comm::protocol::FrameType::ConsecutiveFrame,
                               frame.stdid);
                // extid low byte for consecutive frames used as index
                // index = number of the fragment (1-based). Use integer division by max_payload.
                uint8_t index = static_cast<uint8_t>(
                    send_buffer.used_send_buffer_length / max_payload);
                set_index(index, frame.extid);
            }

            // message id is always stored in extid upper byte
            set_message_id(send_buffer.send_message_id, frame.extid);
            set_source(source, frame.stdid);
            set_destination(send_buffer.send_destination, frame.stdid);

            // copy payload
            memcpy(
                frame.payload,
                &send_buffer
                     .send_message_buffer[send_buffer.used_send_buffer_length],
                chunk);
            frame.length = chunk;

            // advance pointer
            send_buffer.used_send_buffer_length += chunk;

            // return true if more fragments remain
            return send_buffer.used_send_buffer_length <
                   send_buffer.meta.payload_length;
        }

        template <size_t MaxMessageSize>
        bool CAN2TP<MaxMessageSize>::process_segment_frame(
            const CAN2BFrame& frame) {
            switch (get_frame_type(frame.stdid)) {
                case comm::protocol::FrameType::FirstFrame: {
                    const size_t total_len = get_length(frame.extid);

                    if (total_len > MaxMessageSize) {
                        failure_message =
                            "Detected first-frame total length exceeds buffer.";
                        return false;
                    }

                    if (total_len <= MIN_SEGMENT_MESSAGE_LENGTH) {
                        failure_message =
                            "Detected first-frame total length is less than "
                            "minimum segmentation length.";
                        return false;
                    }

                    receive_buffer.used_receive_buffer_length = frame.length;
                    memcpy(receive_buffer.receive_message_buffer, frame.payload,
                           frame.length);

                    // update receive meta
                    receive_buffer.meta.source = get_source(frame.stdid);
                    receive_buffer.meta.destination =
                        get_destination(frame.stdid);
                    receive_buffer.meta.payload_length = total_len;
                    receive_buffer.meta.message_id =
                        get_message_id(frame.extid);

                    return true;
                }
                case comm::protocol::FrameType::ConsecutiveFrame: {
                    // TODO: Add checks for source, destination, message_id match
                    // TODO: verify ordering using the index field (get_index(frame.extid))
                    const size_t remaining =
                        (receive_buffer.meta.payload_length >
                         receive_buffer.used_receive_buffer_length)
                            ? (receive_buffer.meta.payload_length -
                               receive_buffer.used_receive_buffer_length)
                            : 0;

                    if (remaining == 0) {
                        failure_message =
                            "Received consecutive frame but no message is "
                            "being reassembled.";
                        return false;
                    }

                    if (remaining < frame.length) {
                        failure_message =
                            "Received consecutive frame exceeds expected "
                            "message length.";
                        return false;
                    }

                    const size_t to_copy =
                        std::min(remaining, static_cast<size_t>(frame.length));
                    if (to_copy > 0) {
                        memcpy(&receive_buffer.receive_message_buffer
                                    [receive_buffer.used_receive_buffer_length],
                               frame.payload, to_copy);
                        receive_buffer.used_receive_buffer_length += to_copy;
                    }

                    return true;
                }
                default:
                    ASSERT(false,
                           "Invalid frame type for segment frame processing");
            }
        }

        template <size_t MaxMessageSize>
        bool CAN2TP<MaxMessageSize>::get_reassembled_message(
            void* message_received, protocol::TopicMessageMeta& message_meta) {
            if (receive_buffer.used_receive_buffer_length >
                receive_buffer.meta.payload_length) {
                failure_message =
                    "Reassembled message length exceeds expected length.";
                return false;
            }

            if (receive_buffer.used_receive_buffer_length ==
                receive_buffer.meta.payload_length) {
                memcpy(message_received, receive_buffer.receive_message_buffer,
                       receive_buffer.meta.payload_length);
                message_meta = receive_buffer.meta;
                return true;
            }

            failure_message = "No complete message reassembled.";
            return false;
        }

        template <size_t MaxMessageSize>
        [[nodiscard]] bool CAN2TP<MaxMessageSize>::get_single_frame(
            uint8_t* message, size_t message_length, uint8_t destination,
            CAN2BFrame& frame) {
            // Check if message fits in a single frame
            ASSERT(message_length <= sizeof(frame.payload),
                   "Message too large for single frame");
            ASSERT(message_length > 0, "Message length must be greater than 0");

            // Clear frame
            frame.stdid = 0;
            frame.extid = 0;

            // Set frame type and addressing
            set_frame_type(comm::protocol::FrameType::SingleFrame, frame.stdid);
            set_source(source, frame.stdid);
            set_destination(destination, frame.stdid);

            // For single frames, length goes in extid low byte
            set_length(message_length, frame.extid);

            // Copy payload
            memcpy(frame.payload, message, message_length);
            frame.length = message_length;

            return true;
        }

        template <size_t MaxMessageSize>
        [[nodiscard]] bool CAN2TP<MaxMessageSize>::process_single_frame(
            const CAN2BFrame& frame, void* message_received) {
            ASSERT(message_received != nullptr,
                   "message_received pointer is null");

            // Verify frame type
            if (get_frame_type(frame.stdid) !=
                comm::protocol::FrameType::SingleFrame) {
                failure_message =
                    "Detected invalid frame type when processing single frame.";
                return false;
            }

            // Get length from extid
            const size_t length = get_length(frame.extid);

            if (length <= 0 || length > sizeof(frame.payload) ||
                length > frame.length) {
                failure_message =
                    "Detected invalid length when processing single frame.";
                return false;
            }

            // Copy payload to output buffer
            memcpy(message_received, frame.payload, length);

            return true;
        }

        template <size_t MaxMessageSize>
        bool CAN2TP<MaxMessageSize>::get_control_flow_frame(
            CAN2BFrame& frame, ControlFlowID flow_id, uint8_t destination,
            void* params) {
            (void)
                params;  // unused for all currenttly supported control flow frames.
            // Clear frame
            frame.stdid = 0;
            frame.extid = 0;
            frame.length = 0;  // Control flow frames carry no payload

            // Set frame type and addressing
            set_frame_type(comm::protocol::FrameType::ControlFlow, frame.stdid);
            set_source(source, frame.stdid);
            set_destination(destination, frame.stdid);

            // Store control flow ID in the lower byte of extid
            switch (flow_id) {
                case ControlFlowID::Ping:
                    set_length(static_cast<size_t>(ControlFlowID::Ping),
                               frame.extid);
                    break;
                case ControlFlowID::Pong:
                    set_length(static_cast<size_t>(ControlFlowID::Pong),
                               frame.extid);
                    break;
                default:
                    ASSERT(false,
                           "Attempting to get invalid control flow frame with "
                           "unknown ID.");
                    return false;
            }
            return true;
        }

        template <size_t MaxMessageSize>
        bool CAN2TP<MaxMessageSize>::process_control_flow_frame(
            const CAN2BFrame& frame, void* message_received) {
            ASSERT(message_received != nullptr,
                   "message_received pointer is null");
            // Verify frame type
            if (get_frame_type(frame.stdid) !=
                comm::protocol::FrameType::ControlFlow) {
                failure_message =
                    "Detected invalid frame type when processing control flow "
                    "frame.";
                return false;
            }

            // Get control flow ID from lower byte of extid
            auto* flow_id = static_cast<ControlFlowID*>(message_received);
            const uint8_t received_flow_id =
                static_cast<uint8_t>(get_length(frame.extid));

            switch (received_flow_id) {
                case static_cast<uint8_t>(ControlFlowID::Ping):
                    *flow_id = ControlFlowID::Ping;
                    break;
                case static_cast<uint8_t>(ControlFlowID::Pong):
                    *flow_id = ControlFlowID::Pong;
                    break;
                default:
                    return false;
            }

            return true;
        }

        template <size_t MaxMessageSize>
        protocol::TopicMessageMeta
        CAN2TP<MaxMessageSize>::parse_meta_from_headers(uint32_t stdid,
                                                        uint32_t extid) const {
            protocol::TopicMessageMeta meta;
            meta.source = get_source(stdid);
            meta.destination = get_destination(stdid);
            meta.message_id = get_message_id(extid);
            meta.payload_length = get_length(extid);
            return meta;
        }
    }  // namespace v1
}  // namespace comm::can2_tp
#endif
