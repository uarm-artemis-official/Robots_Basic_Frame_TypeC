#include "uc_uartv1.hpp"
#include "uarm_lib.hpp"

namespace uc_uart {
    namespace v1 {
        UC_UARTV1::UC_UARTV1(Config _config, MW_UART::IUART& _uart,
                             MW_UART::Peripheral _peripheral)
            : config(_config), uart(_uart), peripheral(_peripheral) {}

        void UC_UARTV1::send_data_impl(const uint16_t id, const uint8_t* data,
                                       size_t length) {
            check(length <= 8,
                  "Data frame payload cannot be larger than 8 bytes.");
            check(id <= 0x03FF, "Data frame ID is only 10 bits large.");
            ASSERT(length > 0 && data != nullptr,
                   "Data pointer cannot be null with non-zero length.");
            uint8_t dataframe[MAX_DATA_MESSAGE_LENGTH];
            dataframe[0] = FIRST_BYTE;
            uint16_t metadata = (static_cast<uint16_t>(FrameType::Data) << 14) |
                                ((id & 0x03FF) << 4) | (length & 0x000F);
            memcpy(&dataframe[1], &metadata, sizeof(uint16_t));
            memcpy(&dataframe[3], data, length * sizeof(uint8_t));

            uint16_t checksum = calculate_checksum(dataframe, 3 + length);
            memcpy(&dataframe[3 + length], &checksum, sizeof(uint16_t));

            size_t message_length = 3 + length + 2;
            uart.send_data(peripheral, dataframe, message_length, 0);
        }

        void UC_UARTV1::send_control_flow_impl(const uint16_t id,
                                               ControlFlowID control_id,
                                               void* body, size_t length) {
            check(id <= 0x03FF, "Control flow ID is only 10 bits large.");
            uint8_t control_flow_frame[MAX_CONTROL_FLOW_MESSAGE_LENGTH];
            control_flow_frame[0] = FIRST_BYTE;
            uint16_t metadata = (static_cast<uint16_t>(FrameType::Data) << 14) |
                                ((id & 0x03FF) << 4) |
                                (static_cast<uint16_t>(control_id) & 0x000F);
            memcpy(&control_flow_frame[1], &metadata, sizeof(uint16_t));

            if (body != nullptr && length > 0) {
                memcpy(&control_flow_frame[3], body, length * sizeof(uint8_t));
            }

            uint16_t checksum = calculate_checksum(control_flow_frame, 3);
            memcpy(&control_flow_frame[3 + length], &checksum,
                   sizeof(uint16_t));
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

            FrameType frame_type = static_cast<FrameType>(buffer[1] >> 6);

            switch (frame_type) {
                case FrameType::Data: {
                    size_t payload_length = buffer[2] & 0x0F;
                    if (payload_length > 8 || length < (3 + payload_length + 2))
                        return false;
                    memcpy(dst, &buffer[3], payload_length);
                    return true;
                }
                case FrameType::ControlFlow: {
                    uint8_t control_flow_num = buffer[2] & 0x0F;
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
            uint16_t calc_checksum = calculate_checksum(data, length - 4);
            uint16_t message_checksum =
                (data[length - 2] << 8) | data[length - 1];
            if (calc_checksum != message_checksum)
                return false;

            uint8_t sof = data[0] >> 4;
            uint8_t protocol_version = data[0] & 0x0F;

            if (sof != 0x7 || protocol_version != 0x1)
                return false;

            FrameType frame_type = static_cast<FrameType>(data[1] >> 6);
            // uint16_t identifier = ((data[1] & 0x3F) | (data[2] & 0xF0)) >> 4;
            switch (frame_type) {
                case FrameType::Data: {
                    uint8_t payload_length = data[2] & 0x0F;
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