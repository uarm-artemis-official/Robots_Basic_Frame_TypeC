#ifndef __CAN_ENCODING_HPP
#define __CAN_ENCODING_HPP

#include <cstddef>
#include <cstdint>
#include "comm_protocol_interface.hpp"

namespace comm {
    namespace can2_tp {
        // TODO: Add documentation about the protocol format and design.

        inline namespace v1 {
            constexpr size_t MAX_CAN2TP_MESSAGE_LENGTH = 2048;
            constexpr size_t MIN_SEGMENT_MESSAGE_LENGTH = 9;

            // TODO: Use this instead of sizeof or magic numbers.
            constexpr size_t FRAME_PAYLOAD_LENGTH = 8;

            struct CAN2BFrame {
                uint32_t stdid;
                uint32_t extid;
                uint8_t payload[8];
                size_t length;
            };

            enum struct ControlFlowID { Ping = 0x0, Pong = 0x1 };
            /**
         * These are utility functions to set and get various fields
         * of the CAN2TP protocol identifiers (stdid and extid).
         * Their implementations are relatively naive bit manipulations.
         * There is some error checking in the setters to avoid invalid values.
         */
            // TODO: Add error handling/logging for invalid setter inputs.
            // inline void set_frame_type(protocol::FrameType frame_type,
            //                            uint32_t& stdid);
            // inline protocol::FrameType get_frame_type(uint32_t stdid);

            // inline void set_destination(uint8_t destination, uint32_t& stdid);
            // inline uint8_t get_destination(uint32_t stdid);

            // inline void set_source(uint8_t source, uint32_t& stdid);
            // inline uint8_t get_source(uint32_t stdid);

            // inline void set_index(uint8_t index, uint32_t& extid);
            // inline uint8_t get_index(uint32_t extid);

            // inline void set_length(size_t length, uint32_t& extid);
            // inline size_t get_length(uint32_t extid);

            // inline void set_message_id(uint8_t message_id, uint32_t& extid);
            // inline uint8_t get_message_id(uint32_t extid);

            /**
         * @brief Transport Protocol for sending message center topic messages over CAN2.0B.
         * 
         * This is a custom transport layer protocol designed to work over CAN2.0B frames.
         * It is inspired by ISO-TP but tailored for specific use cases in the UARM project.
         * This class provides methods for segmenting and reassembling messages, and 
         * sending/parsing flow control.
         * 
         * It has the following features:
         *  - Segmentation of large messages into multiple CAN2.0B frames.
         *  - Reassembly of messages from received CAN2.0B frames.
         *  - Flow control to determine connection status with different destinations (ping/pong).
         *  
         * Only one segmented message can be reassembled at a time. New messages will overwrite
         * the previous message being reassembled. However, control flow frames and single message
         * frames can be processed at any time without affecting reassembly of another message.
         * 
         * @note this class does not handle sending or receiving CAN2.0B frames directly. It
         * is expected that the user of this class will handle CAN2.0B frame transmission and
         * reception, and call the appropriate methods in this class to process them.
         */
            template <size_t MaxMessageSize>
            class CAN2TP {
                static_assert(
                    MaxMessageSize <= MAX_CAN2TP_MESSAGE_LENGTH,
                    "MaxMessageSize must be <= MAX_CAN2TP_MESSAGE_LENGTH");

               private:
                struct SendBuffer {
                    uint8_t send_message_buffer[MaxMessageSize];
                    size_t used_send_buffer_length;
                    protocol::TopicMessageMeta meta;
                } send_buffer;

                struct ReceiveBuffer {
                    uint8_t receive_message_buffer[MaxMessageSize];
                    size_t used_receive_buffer_length;
                    protocol::TopicMessageMeta meta;
                } receive_buffer;

                uint8_t source;  // source of this CAN2TP instance

                // Debugging aid describing why a method failed (i.e. returned false).
                // NOTE: Usage errors will cause a runtime error and will not populate
                // the failure message.
                const char* failure_message = "";

               public:
                CAN2TP(uint8_t _source) : source(_source) {};

                // TODO: Possibly change the erroring behavior of various methods and add
                // documentation about it in the can2_tp.hpp interface.

                /**
             * @brief Set the message to be sent using CAN2TP.
             * 
             * The message must be large enough for segmentation (check
             * MIN_SEGMENT_MESSAGE_LENGTH), otherwise, please use
             * get_single_frame().
             * This function segments the message into CAN2.0B frames
             * according to the CAN2TP protocol. Message is copied into 
             * an internal buffer for segmentation. Thus message pointer
             * can be freed or go out of scope after this call.
             * 
             * @param message Pointer to the message data to send.
             * @param length Length of the message to send.
             * @param message_id Identifier for the message.
             * @param destination Identifier for the message destination.
             */
                void set_send_message(const uint8_t* message, size_t length,
                                      uint8_t message_id, uint8_t destination);

                /**
             * @brief Checks if there is currently a message in send_buffer that
             * has not finished sending.
             * 
             * @return true if there is a message, otherwise false.
             */
                bool is_sending_message();

                /**
             * @brief Get the next fragment of a segmented message to send.
             * 
             * stdid and extid will be populated with the appropriate identifiers
             * as well as the payload and length of the next fragment.
             * 
             * @param frame Reference to a CAN2BFrame structure to populate with the next fragment.
             * 
             * @return true if there are more fragments to send, false if all fragments have been sent.
             */
                [[nodiscard]] bool get_next_send_fragment(CAN2BFrame& frame);

                /**
             * @brief Process a received segment frame (First Frame or Consecutive Frame).
             * 
             * This function handles reassembling messages from received
             * segment frames according to the CAN2TP protocol. If the message
             * is fully reassembled, use get_reassembled_message() to retrieve it.
             * 
             * @param frame Reference to the received CAN2BFrame.
             * 
             * @return true if the segment frame was processed successfully, false otherwise.
             */
                [[nodiscard]] bool process_segment_frame(
                    const CAN2BFrame& frame);

                /**
             * @brief Retrieve the fully reassembled message after processing segment frames.
             * 
             * This function copies the reassembled message into the provided
             * destination buffer.
             * 
             * @param message_received Pointer to the destination buffer to copy the reassembled message.
             * 
             * @return true if a complete message was reassembled and copied, false otherwise.
             */
                [[nodiscard]] bool get_reassembled_message(
                    void* message_received,
                    protocol::TopicMessageMeta& message_meta);

                /**
             * @brief Creates a single frame if the message fits within single frame limits.
             * 
             * Single frames can carry up to 8 bytes of data. Messages larger than that
             * will have to be fragmented, please use set_send_message() and get_next_send_fragment()
             * methods for that.
             * 
             * @param message Pointer to the message data to send.
             * @param message_length Length of the message to send.
             * @param destination Identifier for the message destination.
             * @param frame Reference to a CAN2BFrame structure to populate with the single frame.
             * 
             * @return true if the message was encoded as a single frame, false if it exceeds single frame limits.
             */
                [[nodiscard]] bool get_single_frame(uint8_t* message,
                                                    size_t message_length,
                                                    uint8_t destination,
                                                    CAN2BFrame& frame);

                /**
             * @brief Process single frame message and copies them to the message_received buffer.
             * 
             * @param frame Reference to the received CAN2BFrame.
             * @param message_received Pointer to the buffer to copy the processed message.
             * 
             * @return true if the single frame was processed successfully, false otherwise.
             */
                [[nodiscard]] bool process_single_frame(const CAN2BFrame& frame,
                                                        void* message_received);

                /**
             * @brief Create a control flow frame.
             * 
             * This function creates a control flow frame. Currently, it supports
             * only Ping and Pong control flow types. However, this can be extended
             * in the future to support more control flow types as needed.
             * 
             * @param frame Reference to a CAN2BFrame structure to populate with the control flow frame.
             * @param flow_id The type of control flow frame to create.
             * @param params Pointer to additional parameters for frame creation.
             * 
             * @return true if the control flow frame was created successfully, false otherwise.
             */
                bool get_control_flow_frame(CAN2BFrame& frame,
                                            ControlFlowID flow_id,
                                            uint8_t destination, void* params);

                /**
             * @brief Process a received control flow frame.
             * 
             * This function handles processing of received control flow frames.
             * The result depends on the control flow type determined from the frame.
             * Currently, there is only ping and pong control flow frames, therefore,
             * message_received will be populated accordingly with an enum value of
             * either ControlFlowID::Ping or ControlFlowID::Pong.
             * 
             * @param frame Reference to the received CAN2BFrame.
             * @param message_received Pointer to the buffer to copy the processed message.
             * 
             * @return true if the control flow frame was processed successfully, false otherwise.
             */
                bool process_control_flow_frame(const CAN2BFrame& frame,
                                                void* message_received);

                protocol::TopicMessageMeta parse_meta_from_headers(
                    uint32_t stdid, uint32_t extid) const;
            };
        }  // namespace v1
    }  // namespace can2_tp

}  // namespace comm

#ifndef __CAN2_TP_IPP
#include "can2_tp.ipp"
#endif

#endif