#include "uc_uartv1.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "mocks/middleware_mocks.hpp"

namespace {

    using namespace uc_uart::v1;
    using ::testing::_;
    using ::testing::DoAll;
    using ::testing::Return;
    using ::testing::SetArrayArgument;

    class UC_UARTV1Test : public ::testing::Test {
       protected:
        MockIUART mock_uart;
        MW_UART::Peripheral test_peripheral = MW_UART::Peripheral::UART1;
        UC_UARTV1 uartv1 {Config::Robust, mock_uart, test_peripheral};
    };

    TEST_F(UC_UARTV1Test, SendDataCallsUARTSend) {
        uint8_t data[4] = {0x11, 0x22, 0x33, 0x44};
        EXPECT_CALL(mock_uart, send_data(test_peripheral, _, _, 0)).Times(1);
        uartv1.send_data_impl(0x01, data, 4);
    }

    TEST_F(UC_UARTV1Test, SendControlFlowCallsUARTSend) {
        EXPECT_CALL(mock_uart, send_data(test_peripheral, _, _, 0)).Times(1);
        uartv1.send_control_flow_impl(0x02, ControlFlowID::Ping, nullptr, 0);
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageValidDataFrame) {
        // Prepare a valid data frame
        uint8_t buffer[9] = {0};
        buffer[0] = FIRST_BYTE;
        set_frame_type(buffer, FrameType::Data);
        set_identifier(buffer, 0x01);
        set_payload_length(buffer, 0x04);
        buffer[3] = 0xAA;
        buffer[4] = 0xBB;
        buffer[5] = 0xCC;
        buffer[6] = 0xDD;
        uint16_t checksum = uartv1.calculate_checksum(buffer, 7);
        memcpy(&buffer[7], &checksum, sizeof(uint16_t));

        uint8_t dst[4] = {0};
        bool result = uartv1.process_receive_message_impl(dst, buffer, 9);
        EXPECT_TRUE(result);
        EXPECT_EQ(dst[0], 0xAA);
        EXPECT_EQ(dst[1], 0xBB);
        EXPECT_EQ(dst[2], 0xCC);
        EXPECT_EQ(dst[3], 0xDD);
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageValidControlFlowFrame) {
        uint8_t buffer[5] = {0};
        buffer[0] = FIRST_BYTE;
        set_frame_type(buffer, FrameType::ControlFlow);
        set_identifier(buffer, 0x02);
        set_control_flow_id(buffer, static_cast<uint8_t>(ControlFlowID::Ping));
        uint16_t checksum = uartv1.calculate_checksum(buffer, 3);
        memcpy(&buffer[3], &checksum, sizeof(uint16_t));

        uint8_t dst[1] = {0};
        bool result = uartv1.process_receive_message_impl(dst, buffer, 5);
        EXPECT_TRUE(result);
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageInvalidChecksum) {
        uint8_t buffer[9] = {0};
        buffer[0] = FIRST_BYTE;
        set_frame_type(buffer, FrameType::Data);
        set_identifier(buffer, 0x01);
        set_payload_length(buffer, 0x04);
        buffer[3] = 0xAA;
        buffer[4] = 0xBB;
        buffer[5] = 0xCC;
        buffer[6] = 0xDD;
        // Wrong checksum
        uint16_t checksum = 0xFFFF;
        memcpy(&buffer[7], &checksum, sizeof(uint16_t));

        uint8_t dst[4] = {0};
        bool result = uartv1.process_receive_message_impl(dst, buffer, 9);
        EXPECT_FALSE(result);
    }

    TEST_F(UC_UARTV1Test, IsStartOfFrameReturnsTrueForValidSOF) {
        uint8_t buffer[] = {FIRST_BYTE};
        EXPECT_TRUE(uartv1.is_start_of_frame_impl(buffer, 1));
    }

    TEST_F(UC_UARTV1Test, IsStartOfFrameReturnsFalseForInvalidSOF) {
        uint8_t buffer[] = {0x00};
        EXPECT_FALSE(uartv1.is_start_of_frame_impl(buffer, 1));
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageTooSmallFrameReturnsFalse) {
        uint8_t buffer[2] = {FIRST_BYTE, 0x01};
        uint8_t dst[1] = {0};
        bool result = uartv1.process_receive_message_impl(dst, buffer, 2);
        EXPECT_FALSE(result);
    }

    TEST_F(UC_UARTV1Test, SendDataImplNullBufferTriggersAssert) {
        EXPECT_DEATH(uartv1.send_data_impl(0x01, nullptr, 4),
                     "Data pointer cannot be null");
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageImplNullBufferTriggersAssert) {
        uint8_t dst[1] = {0};
        EXPECT_DEATH(uartv1.process_receive_message_impl(dst, nullptr, 5),
                     "Buffer cannot be null");
    }

    TEST_F(UC_UARTV1Test, ProcessReceiveMessageImplNullDstTriggersAssert) {
        uint8_t buffer[5] = {FIRST_BYTE, 0x01, 0x21, 0x00, 0x00};
        EXPECT_DEATH(uartv1.process_receive_message_impl(nullptr, buffer, 5),
                     "Destination must be non-null");
    }
}  // namespace