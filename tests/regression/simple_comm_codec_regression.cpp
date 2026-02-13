// tests/regression/simple_comm_codec_regression.cpp
#include <gtest/gtest.h>
#include <array>
#include <random>

#include "../../lib/Middleware/middleware_types.hpp"
#include "simple_comm.hpp"

using namespace simple_comm::v1;

TEST(SimpleCommCodecRegression, RoundtripManyRandom) {
    std::mt19937 rng(12345);
    std::uniform_int_distribution<int> size_d(1, 8);
    std::uniform_int_distribution<int> byte_d(0, 255);

    for (int iter = 0; iter < 200; ++iter) {
        SimpleMessage in {};
        in.source = static_cast<uint8_t>(byte_d(rng) & 0x0F);
        in.destination = static_cast<uint8_t>(byte_d(rng) & 0x0F);
        in.id = static_cast<uint8_t>(byte_d(rng) & 0xFF);
        in.payload_size = static_cast<uint8_t>(size_d(rng));
        for (size_t i = 0; i < in.payload_size; ++i)
            in.payload[i] = std::byte {static_cast<uint8_t>(byte_d(rng))};

        // CAN roundtrip
        MW_CAN::CANFrame frame {};
        SimpleCommCodec<MW_CAN::CANFrame>::to_can_message(in, frame);
        SimpleMessage out {};
        EXPECT_TRUE(
            SimpleCommCodec<MW_CAN::CANFrame>::from_can_message(frame, out));
        EXPECT_EQ(out.payload_size, in.payload_size);

        // Corrupt a random byte in frame payload and ensure detection if it affects magic/dlc
        MW_CAN::CANFrame corrupted = frame;
        if (corrupted.dlc > 0) {
            size_t idx = rng() % corrupted.dlc;
            corrupted.payload[idx] = static_cast<std::byte>(
                static_cast<uint8_t>(corrupted.payload[idx]) ^ 0xFF);
            // payload corruption doesn't change header magic so decoding still succeeds
            SimpleMessage out2 {};
            EXPECT_TRUE(SimpleCommCodec<MW_CAN::CANFrame>::from_can_message(
                corrupted, out2));
        }

        // UART roundtrip
        std::array<std::byte, UART_MAX_MESSAGE_SIZE> buf {};
        size_t out_len = 0;
        SimpleCommCodec<MW_CAN::CANFrame>::to_uart_bytes(
            in, std::span(buf.data(), buf.size()), out_len);
        SimpleMessage outu {};
        EXPECT_TRUE(SimpleCommCodec<MW_CAN::CANFrame>::from_uart_bytes(
            std::span(buf.data(), out_len), outu));

        // Corrupt checksum byte and expect failure
        buf[out_len - 1] = std::byte {static_cast<uint8_t>(
            static_cast<uint8_t>(buf[out_len - 1]) ^ 0xAB)};
        EXPECT_FALSE(SimpleCommCodec<MW_CAN::CANFrame>::from_uart_bytes(
            std::span(buf.data(), out_len), outu));
    }
}
