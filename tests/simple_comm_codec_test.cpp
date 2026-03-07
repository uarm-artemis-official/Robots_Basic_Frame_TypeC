// tests/simple_comm_codec_test.cpp
#include <gtest/gtest.h>
#include <array>
#include <cstdint>
#include <vector>

#include "middleware_types.hpp"
#include "simple_comm_utils.hpp"

using namespace simple_comm::v1;

namespace {
    TEST(SimpleCommCodecTest, CanRoundtrip) {
        SimpleMessage in {};
        in.source = 0x3;
        in.destination = 0x4;
        in.id = 0x42;
        in.payload_size = 3;
        in.payload[0] = std::byte {0x10};
        in.payload[1] = std::byte {0x20};
        in.payload[2] = std::byte {0x30};

        MW_CAN::CANFrame frame {};
        SimpleCommCodec<MW_CAN::CANFrame>::to_can_message(in, frame);

        SimpleMessage out {};
        bool ok =
            SimpleCommCodec<MW_CAN::CANFrame>::from_can_message(frame, out);
        EXPECT_TRUE(ok);
        EXPECT_EQ(out.source, in.source);
        EXPECT_EQ(out.destination, in.destination);
        EXPECT_EQ(out.id, in.id);
        EXPECT_EQ(out.payload_size, in.payload_size);
        for (size_t i = 0; i < out.payload_size; ++i) {
            EXPECT_EQ(out.payload[i], in.payload[i]);
        }
    }

    TEST(SimpleCommCodecTest, UartRoundtripAndChecksum) {
        SimpleMessage in {};
        in.source = 0x2;
        in.destination = 0x1;
        in.id = 0x5A;
        in.payload_size = 4;
        in.payload[0] = std::byte {0x01};
        in.payload[1] = std::byte {0x02};
        in.payload[2] = std::byte {0x03};
        in.payload[3] = std::byte {0x04};

        std::array<std::byte, UART_MAX_MESSAGE_SIZE> buf {};
        size_t out_len = 0;
        SimpleCommCodec<MW_CAN::CANFrame>::to_uart_bytes(
            in, std::span(buf.data(), buf.size()), out_len);

        SimpleMessage out {};
        bool ok = SimpleCommCodec<MW_CAN::CANFrame>::from_uart_bytes(
            std::span(buf.data(), out_len), out);
        EXPECT_TRUE(ok);
        EXPECT_EQ(out.payload_size, in.payload_size);
        for (size_t i = 0; i < out.payload_size; ++i)
            EXPECT_EQ(out.payload[i], in.payload[i]);

        // Corrupt a byte in the payload and ensure checksum fails
        buf[UART_HEADER_SIZE + 1] = std::byte {static_cast<uint8_t>(
            static_cast<uint8_t>(buf[UART_HEADER_SIZE + 1]) ^ 0xFF)};
        bool ok2 = SimpleCommCodec<MW_CAN::CANFrame>::from_uart_bytes(
            std::span(buf.data(), out_len), out);
        EXPECT_FALSE(ok2);
    }

    TEST(SimpleCommCodecTest, CanCorruptionDetected) {
        SimpleMessage in {};
        in.source = 0x0;
        in.destination = 0xF;
        in.id = 0x7F;
        in.payload_size = 2;
        in.payload[0] = std::byte {0xAA};
        in.payload[1] = std::byte {0xBB};

        MW_CAN::CANFrame frame {};
        SimpleCommCodec<MW_CAN::CANFrame>::to_can_message(in, frame);

        // flip the magic bits so it's no longer a simple-comm frame
        frame.eid ^= (1u << 26);
        SimpleMessage out {};
        bool ok =
            SimpleCommCodec<MW_CAN::CANFrame>::from_can_message(frame, out);
        EXPECT_FALSE(ok);
    }
}  // namespace
