#include "uc_uartv1.hpp"
#include "uarm_lib.hpp"

namespace uc_uart {
    namespace v1 {
        UC_UARTV1::UC_UARTV1(Config _config, MW_UART::IUART& _uart)
            : config(_config), uart(_uart) {}

        void UC_UARTV1::send_data_impl(const uint16_t id, const uint8_t* data,
                                       size_t length) {}

        void UC_UARTV1::send_control_flow_impl(const uint16_t id,
                                               ControlFlowID control_id,
                                               void* body) {}

        bool UC_UARTV1::process_receive_message_impl(void* dst, uint8_t* buffer,
                                                     size_t length) {}

        uint16_t UC_UARTV1::calculate_checksum(const uint8_t* data,
                                               size_t length) {
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