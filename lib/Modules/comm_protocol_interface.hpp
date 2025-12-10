#ifndef __PROTOCOL_HPP
#define __PROTOCOL_HPP

// TODO: Consolidate interface and types into existing communication protocols.
namespace comm {
    namespace protocol {
        inline namespace v1 {

            struct TopicMessageMeta {
                uint8_t source;
                uint8_t destination;
                size_t payload_length;
                uint8_t message_id;

                bool operator==(const TopicMessageMeta& other) const {
                    return source == other.source &&
                           destination == other.destination &&
                           payload_length == other.payload_length &&
                           message_id == other.message_id;
                }
            };

            enum class FrameType {
                Invalid = 0x0,
                Data = 0x1,
                ControlFlow = 0x2,
                SingleFrame = 0x3,
                FirstFrame = 0x4,
                ConsecutiveFrame = 0x5,
            };

            class Protocol {
               public:
                virtual FrameType get_frame_type(
                    const uint8_t* frame) const = 0;
                virtual TopicMessageMeta parse_meta_from_headers(
                    const uint8_t* header) const = 0;
                virtual bool is_valid_frame(const uint8_t* frame,
                                            size_t frame_length) const = 0;
                virtual bool process_frame(const uint8_t* frame,
                                           size_t frame_length) = 0;
                virtual bool get_latest_message(uint8_t* message_buffer,
                                                size_t& message_buffer_length,
                                                TopicMessageMeta& meta) = 0;
                virtual bool set_send_message(uint8_t* payload,
                                              TopicMessageMeta meta) = 0;
                virtual size_t get_next_send_frame(uint8_t* dst) = 0;
            };
        }  // namespace v1

    }  // namespace protocol
}  // namespace comm

#endif