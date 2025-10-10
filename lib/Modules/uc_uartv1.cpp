#include "uc_uartv1.hpp"
#include <cstring>
#include "uarm_lib.hpp"

namespace uc_uart {
    namespace v1 {
        UC_UARTV1::UC_UARTV1(Config _config, MW_UART::IUART& _uart,
                             MW_UART::Peripheral _peripheral)
            : config(_config), uart(_uart), peripheral(_peripheral) {}

        void UC_UARTV1::send_data_impl(const uint8_t id, const uint8_t* data,
                                       size_t length) {
            check(length <= 8,
                  "Data frame payload cannot be larger than 8 bytes.");
            ASSERT(length > 0 && data != nullptr,
                   "Data pointer cannot be null with non-zero length.");
            uint8_t dataframe[MAX_DATA_MESSAGE_LENGTH];
            dataframe[0] = FIRST_BYTE;
            set_frame_type(dataframe, FrameType::Data);
            set_identifier(dataframe, id);
            set_payload_length(dataframe, static_cast<uint8_t>(length));
            memcpy(&dataframe[3], data, length * sizeof(uint8_t));

            uint16_t checksum = calculate_checksum(dataframe, 3 + length);
            memcpy(&dataframe[3 + length], &checksum, sizeof(uint16_t));

            size_t message_length = 3 + length + 2;
            uart.send_data(peripheral, dataframe, message_length, 0);
        }

        void UC_UARTV1::send_control_flow_impl(const uint8_t id,
                                               ControlFlowID control_id,
                                               void* body, size_t length) {
            uint8_t control_flow_frame[MAX_CONTROL_FLOW_MESSAGE_LENGTH];
            control_flow_frame[0] = FIRST_BYTE;
            set_frame_type(control_flow_frame, FrameType::ControlFlow);
            set_identifier(control_flow_frame, id);
            set_control_flow_id(control_flow_frame,
                                static_cast<uint8_t>(control_id));

            if (body != nullptr && length > 0) {
                memcpy(&control_flow_frame[3], body, length * sizeof(uint8_t));
            }

            uint16_t checksum = calculate_checksum(control_flow_frame, 3);
            memcpy(&control_flow_frame[3 + length], &checksum,
                   sizeof(uint16_t));

            size_t message_length = 3 + length + 2;
            uart.send_data(peripheral, control_flow_frame, message_length, 0);
        }

        bool UC_UARTV1::process_receive_message_impl(void* dst, uint8_t* buffer,
                                                     size_t length) {
            ASSERT(dst != nullptr, "Destination must be non-null.");
            ASSERT(buffer != nullptr, "Buffer cannot be null");
            // Minimum frame size: SOF+metadata+checksum
            if (length < 5)
                return false;

            // Verify integrity
            if (!verify_message_integrity(buffer, length))
                return false;

            FrameType frame_type = get_frame_type(buffer);

            switch (frame_type) {
                case FrameType::Data: {
                    size_t payload_length = get_payload_length(buffer);
                    if (payload_length > 8 || length < (3 + payload_length + 2))
                        return false;
                    memcpy(dst, &buffer[3], payload_length);
                    return true;
                }
                case FrameType::ControlFlow: {
                    uint8_t control_flow_num = get_control_flow_id(buffer);
                    ControlFlowID control_flow_id =
                        static_cast<ControlFlowID>(control_flow_num);
                    switch (control_flow_id) {
                        case ControlFlowID::Ping:
                            [[fallthrough]];
                        case ControlFlowID::Pong:
                            memcpy(dst, &control_flow_num,
                                   sizeof(control_flow_num));
                            break;
                        default:
                            check(false,
                                  "Received control flow frame with unknown "
                                  "control flow ID.");
                    }
                    return true;
                }
                default:
                    return false;
            }
        }

        bool UC_UARTV1::is_start_of_frame_impl(uint8_t* buffer, size_t length) {
            ASSERT(buffer != nullptr, "buffer cannot be null.");
            if (length == 0)
                return false;
            return buffer[0] == FIRST_BYTE;
        }

        uint16_t UC_UARTV1::calculate_checksum(const uint8_t* data,
                                               size_t length) {
            if (data == nullptr)
                return 0;
            // CRC-16-CCITT (poly 0x1021, initial value 0xFFFF)
            uint16_t crc = 0xFFFF;
            for (size_t i = 0; i < length; ++i) {
                crc ^= (static_cast<uint16_t>(data[i]) << 8);
                for (int j = 0; j < 8; ++j) {
                    if (crc & 0x8000) {
                        crc = (crc << 1) ^ 0x1021;
                    } else {
                        crc <<= 1;
                    }
                }
            }
            return crc;
        }

        bool UC_UARTV1::verify_message_integrity(const uint8_t* data,
                                                 size_t length) {
            if (data == nullptr)
                return false;
            uint16_t calc_checksum = calculate_checksum(data, length - 2);
            uint16_t message_checksum;
            memcpy(&message_checksum, &data[length - 2], sizeof(uint16_t));
            if (calc_checksum != message_checksum)
                return false;

            uint8_t sof = data[0] >> 4;
            uint8_t protocol_version = data[0] & 0x0F;

            if (sof != 0x7 || protocol_version != 0x1)
                return false;

            FrameType frame_type = get_frame_type(data);
            switch (frame_type) {
                case FrameType::Data: {
                    uint8_t payload_length = get_payload_length(data);
                    if (payload_length > 8)
                        return false;
                    break;
                }
                case FrameType::ControlFlow: {
                    // TODO: Implement.
                    break;
                }
                default:
                    return false;
            }
            return true;
        }

        void UC_UARTV1::check(bool cond, const char* msg) {
            if (config == Config::FailFast) {
                ASSERT(cond, msg);
            }
        }
    }  // namespace v1
}  // namespace uc_uart