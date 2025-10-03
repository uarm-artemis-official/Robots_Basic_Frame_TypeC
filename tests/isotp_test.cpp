#include "isotp.hpp"
#include <gtest/gtest.h>

using namespace isotp;

struct DummySend {
    std::vector<std::vector<uint8_t>> sent_frames;
    void operator()(uint8_t* frame, size_t len) {
        sent_frames.emplace_back(frame, frame + len);
    }
};

struct DummyDelay {
    void operator()() {}
};

TEST(ISOTPTest, SingleFrameSendReceive) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    uint8_t msg[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    isotp.set_send_message(msg, 4);
    isotp.tick_send_process();

    ASSERT_EQ(send.sent_frames.size(), 1);
    ASSERT_EQ(send.sent_frames[0][0], 0x04);  // PCI: SingleFrame, length 4
    ASSERT_EQ(send.sent_frames[0][1], 0xAA);
    ASSERT_EQ(send.sent_frames[0][2], 0xBB);
    ASSERT_EQ(send.sent_frames[0][3], 0xCC);
    ASSERT_EQ(send.sent_frames[0][4], 0xDD);

    // Simulate receiving the same frame
    isotp.process_receive_frame(send.sent_frames[0].data(),
                                send.sent_frames[0].size());
    uint8_t recv[4] = {0};
    ASSERT_TRUE(isotp.get_receive_message(recv));
    ASSERT_EQ(recv[0], 0xAA);
    ASSERT_EQ(recv[1], 0xBB);
    ASSERT_EQ(recv[2], 0xCC);
    ASSERT_EQ(recv[3], 0xDD);
}

TEST(ISOTPTest, MultiFrameSendReceive) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    uint8_t msg[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    isotp.set_send_message(msg, 10);
    isotp.tick_send_process();  // First frame
    isotp.tick_send_process();  // Consecutive frames

    // Should have sent 1 first frame and 1 consecutive frame
    ASSERT_GE(send.sent_frames.size(), 2);

    // Simulate receiving first frame
    isotp.process_receive_frame(send.sent_frames[0].data(),
                                send.sent_frames[0].size());
    // Simulate receiving consecutive frame(s)
    for (size_t i = 1; i < send.sent_frames.size(); ++i) {
        isotp.process_receive_frame(send.sent_frames[i].data(),
                                    send.sent_frames[i].size());
    }

    uint8_t recv[10] = {0};
    ASSERT_TRUE(isotp.get_receive_message(recv));
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(recv[i], i);
    }
}

TEST(ISOTPTest, StateTransitions) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    ASSERT_EQ(isotp.get_send_state(), ISOTPSendState::Ready);
    ASSERT_EQ(isotp.get_receive_state(), ISOTPReceiveState::Ready);

    uint8_t msg[2] = {0x11, 0x22};
    isotp.set_send_message(msg, 2);
    ASSERT_EQ(isotp.get_send_state(), ISOTPSendState::Sending);

    isotp.tick_send_process();
    ASSERT_EQ(isotp.get_send_state(), ISOTPSendState::Ready);

    // Simulate receive
    uint8_t frame[3] = {0x02, 0x11, 0x22};
    isotp.process_receive_frame(frame, 3);
    ASSERT_EQ(isotp.get_receive_state(), ISOTPReceiveState::HaveFullMessage);

    uint8_t recv[2] = {0};
    ASSERT_TRUE(isotp.get_receive_message(recv));
    ASSERT_EQ(isotp.get_receive_state(), ISOTPReceiveState::Ready);
}

TEST(ISOTPTest, ResetFSM) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    uint8_t msg[3] = {0x01, 0x02, 0x03};
    isotp.set_send_message(msg, 3);
    isotp.tick_send_process();

    isotp.reset();
    ASSERT_EQ(isotp.get_send_state(), ISOTPSendState::Ready);
    ASSERT_EQ(isotp.get_receive_state(), ISOTPReceiveState::Ready);
}
