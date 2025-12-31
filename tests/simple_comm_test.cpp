// tests/simple_comm_test.cpp
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <vector>

#include "../lib/Middleware/middleware_interfaces.hpp"
#include "../lib/Modules/simple_comm.hpp"
#include "fakes/fake_uart.hpp"

using namespace simple_comm::v1;

namespace {
    static uint16_t compute_checksum(const std::vector<uint8_t>& bytes) {
        uint32_t sum = 0;
        for (uint8_t b : bytes)
            sum += b;
        return static_cast<uint16_t>(sum & 0xFFFF);
    }

    static std::vector<uint8_t> build_uart_frame(
        uint8_t dest, uint8_t src, uint8_t topic,
        const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> v;
        v.push_back(static_cast<uint8_t>(MAGIC_TRIBIT));
        v.push_back(static_cast<uint8_t>((dest << 4) | (src & 0x0F)));
        v.push_back(topic);
        v.push_back(static_cast<uint8_t>(payload.size()));
        v.insert(v.end(), payload.begin(), payload.end());
        uint16_t cks = compute_checksum(v);
        v.push_back(static_cast<uint8_t>(cks & 0xFF));
        v.push_back(static_cast<uint8_t>((cks >> 8) & 0xFF));
        return v;
    }

    static MW_CAN::CANFrame build_can_frame(
        uint8_t dest, uint8_t src, uint8_t topic,
        const std::vector<uint8_t>& payload) {
        MW_CAN::CANFrame f;
        f.sid = (static_cast<uint32_t>(MAGIC_TRIBIT) << 8) |
                ((static_cast<uint32_t>(dest) & 0x0F) << 4) |
                (static_cast<uint32_t>(src) & 0x0F);
        f.eid = (static_cast<uint32_t>(topic) << 11);
        f.dlc = static_cast<uint8_t>(payload.size());
        for (size_t i = 0; i < payload.size() && i < 8; ++i) {
            f.payload[i] = std::byte {payload[i]};
        }
        return f;
    }

    // Use a test fixture to reuse SimpleComm and FakeUART instances
    class SimpleCommTest : public ::testing::Test {
       protected:
        SimpleComm<4> comm;
        FakeUART fake;

        void SetUp() override { /* nothing for now */ }
    };

    TEST_F(SimpleCommTest, UartInitSuccess) {
        fake.enqueue_rx_bytes({0xFF});
        bool res = comm.uart_isr_init(fake);
        EXPECT_FALSE(fake.calls.empty());
        EXPECT_TRUE(res);
        EXPECT_EQ(fake.calls.back().len, 1u);
    }

    TEST_F(SimpleCommTest, CanIsrMessagePending) {
        MW_CAN::CANFrame f = build_can_frame(0x4, 0x3, 0x42, {0x10, 0x20, 0x30});
        comm.can_isr_message_pending(MW_CAN::BUS::CAN_2B, f);
        SimpleMessage m;
        EXPECT_TRUE(comm.get_rx_message(m));
        EXPECT_EQ(m.destination, 0x4);
        EXPECT_EQ(m.source, 0x3);
        EXPECT_EQ(m.topic_id, 0x42);
        EXPECT_EQ(m.payload_size, 3);
        EXPECT_EQ(static_cast<uint8_t>(m.payload[0]), 0x10);
    }

    TEST_F(SimpleCommTest, UartFullMessagePush) {
        // Build a full message: dest=4 src=3 topic=0x42 payload {0x10,0x20,0x30}
        auto frame = build_uart_frame(0x4, 0x3, 0x42, {0x10, 0x20, 0x30});

        // Enqueue all bytes; receive_data will be called in stages and consume from queue
        fake.enqueue_rx_bytes(frame);

        // Arm reception (initial 1 byte)
        ASSERT_TRUE(comm.uart_isr_init(fake));

        // Simulate ISR triggers after each receive completion
        comm.uart_isr_receive_complete(fake, MW_UART::Peripheral::UART1);  // tribit -> request header
        comm.uart_isr_receive_complete(fake, MW_UART::Peripheral::UART1);  // header -> request payload+trailer
        comm.uart_isr_receive_complete(fake, MW_UART::Peripheral::UART1);  // payload+trailer -> process

        SimpleMessage m;
        ASSERT_TRUE(comm.get_rx_message(m));
        EXPECT_EQ(m.destination, 0x4);
        EXPECT_EQ(m.source, 0x3);
        EXPECT_EQ(m.topic_id, 0x42);
        EXPECT_EQ(m.payload_size, 3);
        EXPECT_EQ(static_cast<uint8_t>(m.payload[0]), 0x10);
        EXPECT_EQ(static_cast<uint8_t>(m.payload[1]), 0x20);
        EXPECT_EQ(static_cast<uint8_t>(m.payload[2]), 0x30);
    }
}  // namespace
