#include "isotp.hpp"
#include <gtest/gtest.h>

using namespace isotp;

struct DummySend {
    std::vector<std::vector<uint8_t>> sent_frames;
    void operator()(uint8_t* frame, size_t len) {
        std::vector<uint8_t> new_frame(len);
        memcpy(new_frame.data(), frame, len);
        sent_frames.push_back(new_frame);
    }
};

struct DummyDelay {
    uint32_t requested_delay = 0;

    void operator()(uint32_t ms) { requested_delay = ms; }
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

    uint8_t fc[3];
    fc[0] = 0x30 | static_cast<uint8_t>(isotp::FlowControlFlag::Continue);
    fc[1] = 0;
    fc[2] = 0;
    isotp.process_receive_frame(fc, 3);

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
    ASSERT_EQ(isotp.get_send_state(), ISOTPSendState::SendingSingleOrFirst);

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

TEST(ISOTPTest, MultiFrameSendWithFlowControl) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    // Prepare a message longer than 7 bytes.
    const size_t message_size = 50;
    uint8_t msg[message_size];
    for (size_t i = 0; i < message_size; ++i)
        msg[i] = static_cast<uint8_t>(i);
    isotp.set_send_message(msg, message_size);

    // First tick: should send First Frame
    isotp.tick_send_process();
    ASSERT_EQ(send.sent_frames.size(), 1);
    ASSERT_EQ(send.sent_frames[0][0] >> 4, 0x1);  // First Frame PCI
    ASSERT_EQ(isotp.get_send_state(), isotp::ISOTPSendState::WaitingForControl);

    // Second tick: should wait for Flow Control before sending consecutive frames
    isotp.tick_send_process();
    ASSERT_EQ(send.sent_frames.size(), 1);  // No new frames sent

    // Simulate receiving Flow Control frame with block_size = 2, stmin = 5ms, continue
    uint8_t fc_frame[3] = {
        static_cast<uint8_t>(0x30 |
                             static_cast<uint8_t>(FlowControlFlag::Continue)),
        2, 5};
    isotp.process_receive_frame(fc_frame, 3);

    // Third tick: send a consecutive frame.
    isotp.tick_send_process();
    ASSERT_EQ(send.sent_frames.size(), 2);
    ASSERT_EQ(send.sent_frames[1][0] >> 4, 0x2);
    ASSERT_EQ(delay.requested_delay, 5);
    ASSERT_EQ(isotp.get_send_state(),
              isotp::ISOTPSendState::SendingConsecutive);

    // Fourth tick: send another consecutive frame.
    isotp.tick_send_process();
    ASSERT_EQ(send.sent_frames.size(), 3);
    ASSERT_EQ(send.sent_frames[2][0] >> 4, 0x2);
    ASSERT_EQ(delay.requested_delay, 5);
    ASSERT_EQ(isotp.get_send_state(), isotp::ISOTPSendState::WaitingForControl);

    // Transmit rest of message.
    isotp.process_receive_frame(fc_frame, 3);
    isotp.tick_send_process();
    isotp.tick_send_process();
    isotp.process_receive_frame(fc_frame, 3);
    isotp.tick_send_process();
    isotp.tick_send_process();
    isotp.process_receive_frame(fc_frame, 3);
    isotp.tick_send_process();
    isotp.tick_send_process();  // No frame should be sent.

    ASSERT_EQ(send.sent_frames.size(), 8);

    // Check that all message bytes were sent in consecutive frames.
    // Also check that correct data was sent.
    // First frame.
    size_t total_data_sent = 6;
    ASSERT_EQ(send.sent_frames[0].size(), 8);
    for (size_t i = 0; i < 6; i++)
        ASSERT_EQ(send.sent_frames[0][2 + i], i);

    // Consecutive frames.
    uint8_t next_data = 6;
    for (size_t i = 1; i < send.sent_frames.size(); ++i) {
        ASSERT_TRUE(send.sent_frames[i].size() <= 8);
        total_data_sent += send.sent_frames[i].size() - 1;
        for (size_t j = 0; j < send.sent_frames[i].size() - 1; j++) {
            ASSERT_EQ(send.sent_frames[i][1 + j], next_data++);
        }
    }
    ASSERT_EQ(total_data_sent, message_size);
}

TEST(ISOTPTest, NoSendInReadyState) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    // Prepare a message
    uint8_t msg[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    isotp.set_send_message(msg, 8);

    // Manually set state to Ready
    isotp.set_send_state(ISOTPSendState::Ready);

    // Try to send
    isotp.tick_send_process();

    // No frames should be sent
    ASSERT_EQ(send.sent_frames.size(), 0);
}

TEST(ISOTPTest, NoSendInWaitingForControlState) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    uint8_t msg[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    isotp.set_send_message(msg, 8);

    // Manually set state to WaitingForControl
    isotp.set_send_state(ISOTPSendState::WaitingForControl);

    isotp.tick_send_process();

    ASSERT_EQ(send.sent_frames.size(), 0);
}

TEST(ISOTPTest, NoSendInAbortedState) {
    DummySend send;
    DummyDelay delay;
    ISOTP<DummySend, DummyDelay> isotp(send, delay, ISOTPConfig::FailFast);

    uint8_t msg[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    isotp.set_send_message(msg, 8);

    // Manually set state to Aborted
    isotp.set_send_state(ISOTPSendState::Aborted);

    isotp.tick_send_process();

    ASSERT_EQ(send.sent_frames.size(), 0);
}
