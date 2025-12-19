#include "can2_tp.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace {
    // Test fixture and helpers for CAN2TP tests
    class CAN2TPTest : public ::testing::Test {
       protected:
        static constexpr size_t MaxMessageSize = 64;
        static constexpr uint8_t source = 1;
        comm::can2_tp::CAN2TP<MaxMessageSize> can2tp {source};  // source = 1

        // Build a deterministic payload of length n (0..255 repeating)
        static std::vector<uint8_t> BuildPayload(size_t n) {
            std::vector<uint8_t> v(n);
            for (size_t i = 0; i < n; ++i) v[i] = static_cast<uint8_t>(i & 0xFF);
            return v;
        }

        // Drain all fragments produced by get_next_send_fragment and return them
        static std::vector<comm::can2_tp::CAN2BFrame> DrainFragments(
            comm::can2_tp::CAN2TP<MaxMessageSize>& tp) {
            std::vector<comm::can2_tp::CAN2BFrame> frames;
            comm::can2_tp::CAN2BFrame frame;
            while (tp.get_next_send_fragment(frame)) {
                frames.push_back(frame);
            }
            return frames;
        }

        // Concatenate payloads from frames up to their declared length
        static std::vector<uint8_t> ConcatPayloadsFromFrames(
            const std::vector<comm::can2_tp::CAN2BFrame>& frames) {
            std::vector<uint8_t> out;
            for (const auto& f : frames) {
                out.insert(out.end(), f.payload, f.payload + f.length);
            }
            return out;
        }

        // Simple frame equality on key fields (stdid, extid, length, payload)
        static bool FrameCompare(const comm::can2_tp::CAN2BFrame& a,
                                 const comm::can2_tp::CAN2BFrame& b) {
            if (a.stdid != b.stdid) return false;
            if (a.extid != b.extid) return false;
            if (a.length != b.length) return false;
            for (size_t i = 0; i < a.length; ++i) if (a.payload[i] != b.payload[i]) return false;
            return true;
        }
    };

    // ----- Single-frame tests -----
    TEST_F(CAN2TPTest, GetSingleFrame_HappyPath) {
        auto msg = BuildPayload(5);
        comm::can2_tp::CAN2BFrame frame;
        const uint8_t destination = 2;

        bool result = can2tp.get_single_frame(msg.data(), msg.size(), destination, frame);

        EXPECT_TRUE(result);
        EXPECT_EQ(frame.length, msg.size());
        EXPECT_EQ(comm::can2_tp::get_frame_type(frame.stdid), comm::protocol::FrameType::SingleFrame);
        EXPECT_EQ(comm::can2_tp::get_source(frame.stdid), CAN2TPTest::source);
        EXPECT_EQ(comm::can2_tp::get_destination(frame.stdid), destination);
        EXPECT_EQ(comm::can2_tp::get_length(frame.extid), msg.size());
        for (size_t i = 0; i < msg.size(); ++i) EXPECT_EQ(frame.payload[i], msg[i]);
    }

    TEST_F(CAN2TPTest, ProcessSingleFrame_HappyPath) {
        auto msg = BuildPayload(6);
        comm::can2_tp::CAN2BFrame frame;

        ASSERT_TRUE(can2tp.get_single_frame(msg.data(), msg.size(), 3, frame));

        uint8_t out[8] = {0};
        bool processed = can2tp.process_single_frame(frame, out);
        EXPECT_TRUE(processed);
        for (size_t i = 0; i < msg.size(); ++i) EXPECT_EQ(out[i], msg[i]);
    }

    // ----- Control-flow tests -----
    TEST_F(CAN2TPTest, GetAndProcessControlFlowFrame_PingPong) {
        comm::can2_tp::CAN2BFrame frame_ping;
        ASSERT_TRUE(can2tp.get_control_flow_frame(frame_ping, comm::can2_tp::ControlFlowID::Ping, 4, nullptr));
        EXPECT_EQ(comm::can2_tp::get_frame_type(frame_ping.stdid), comm::protocol::FrameType::ControlFlow);

        comm::can2_tp::ControlFlowID received_id;
        ASSERT_TRUE(can2tp.process_control_flow_frame(frame_ping, &received_id));
        EXPECT_EQ(received_id, comm::can2_tp::ControlFlowID::Ping);

        comm::can2_tp::CAN2BFrame frame_pong;
        ASSERT_TRUE(can2tp.get_control_flow_frame(frame_pong, comm::can2_tp::ControlFlowID::Pong, 4, nullptr));
        ASSERT_TRUE(can2tp.process_control_flow_frame(frame_pong, &received_id));
        EXPECT_EQ(received_id, comm::can2_tp::ControlFlowID::Pong);
    }

    // ----- Segmented send and reassembly tests -----
    TEST_F(CAN2TPTest, SegmentedSendAndReassemble) {
        // Build a message longer than MIN_SEGMENT_MESSAGE_LENGTH (>=9)
        const size_t msg_len = 20;
        auto message = BuildPayload(msg_len);
        const uint8_t message_id = 7;
        const uint8_t destination = 5;

        // Set message to send
        can2tp.set_send_message(message.data(), message.size(), message_id, destination);

        // Per specification, instance should report that it's sending a message
        EXPECT_TRUE(can2tp.is_sending_message());

        // Drain fragments from sender
        std::vector<comm::can2_tp::CAN2BFrame> frames = DrainFragments(can2tp);
        // There should be at least 1 frame
        EXPECT_FALSE(frames.empty());

        // Reconstruct payload from produced frames and compare
        auto reconstructed = ConcatPayloadsFromFrames(frames);
        EXPECT_EQ(reconstructed.size(), message.size());
        for (size_t i = 0; i < message.size(); ++i) EXPECT_EQ(reconstructed[i], message[i]);

        // After draining, sender should no longer be sending
        EXPECT_FALSE(can2tp.is_sending_message());

        // Now simulate receiver: feed frames to a fresh CAN2TP instance
        comm::can2_tp::CAN2TP<MaxMessageSize> receiver{static_cast<uint8_t>(destination)};
        for (const auto& f : frames) {
            // process_segment_frame handles FirstFrame and ConsecutiveFrame
            ASSERT_TRUE(receiver.process_segment_frame(f));
        }

        // Retrieve reassembled message
        uint8_t recv_buf[MaxMessageSize] = {0};
        comm::protocol::TopicMessageMeta meta;
        bool got = receiver.get_reassembled_message(recv_buf, meta);
        EXPECT_TRUE(got);
        EXPECT_EQ(meta.payload_length, message.size());
        for (size_t i = 0; i < message.size(); ++i) EXPECT_EQ(recv_buf[i], message[i]);
    }

    // ----- Header/meta parsing test -----
    TEST_F(CAN2TPTest, ParseMetaFromHeaders_RoundTrip) {
        // Create a single frame and parse headers via parse_meta_from_headers
        auto msg = BuildPayload(4);
        comm::can2_tp::CAN2BFrame frame;
        const uint8_t destination = 9;
        ASSERT_TRUE(can2tp.get_single_frame(msg.data(), msg.size(), destination, frame));

        auto meta = can2tp.parse_meta_from_headers(frame.stdid, frame.extid);
        EXPECT_EQ(meta.source, CAN2TPTest::source);
        EXPECT_EQ(meta.destination, destination);
        EXPECT_EQ(meta.payload_length, msg.size());
    }

    // ----- Edge case tests -----
    TEST_F(CAN2TPTest, GetSingleFrame_LengthZero_Death) {
        uint8_t buf[1] = {0};
        comm::can2_tp::CAN2BFrame frame;
        ASSERT_DEATH(can2tp.get_single_frame(buf, 0, 1, frame), ".*");
    }

    TEST_F(CAN2TPTest, GetSingleFrame_LengthTooLarge_Death) {
        uint8_t buf[9] = {0};
        comm::can2_tp::CAN2BFrame frame;
        ASSERT_DEATH(can2tp.get_single_frame(buf, 9, 1, frame), ".*");
    }

    TEST_F(CAN2TPTest, ProcessSingleFrame_InvalidFrameType_ReturnsFalse) {
        comm::can2_tp::CAN2BFrame cf;
        ASSERT_TRUE(can2tp.get_control_flow_frame(cf, comm::can2_tp::ControlFlowID::Ping, 2, nullptr));
        uint8_t out[8] = {0};
        EXPECT_FALSE(can2tp.process_single_frame(cf, out));
    }

    TEST_F(CAN2TPTest, ProcessSingleFrame_ExtidLengthMismatch_ReturnsFalse) {
        auto msg = BuildPayload(4);
        comm::can2_tp::CAN2BFrame frame;
        ASSERT_TRUE(can2tp.get_single_frame(msg.data(), msg.size(), 3, frame));
        // Corrupt extid length to be larger than frame.length
        comm::can2_tp::set_length(static_cast<size_t>(frame.length + 1), frame.extid);
        uint8_t out[8] = {0};
        EXPECT_FALSE(can2tp.process_single_frame(frame, out));
    }

    TEST_F(CAN2TPTest, GetControlFlowFrame_InvalidFlowID_Death) {
        comm::can2_tp::CAN2BFrame frame;
        auto bad_id = static_cast<comm::can2_tp::ControlFlowID>(0xFF);
        ASSERT_DEATH(can2tp.get_control_flow_frame(frame, bad_id, 1, nullptr), ".*");
    }

    TEST_F(CAN2TPTest, ProcessControlFlowFrame_UnrecognizedID_ReturnsFalse) {
        comm::can2_tp::CAN2BFrame frame;
        ASSERT_TRUE(can2tp.get_control_flow_frame(frame, comm::can2_tp::ControlFlowID::Ping, 1, nullptr));
        // Set extid low byte to an unrecognized ID
        comm::can2_tp::set_length(0xFF, frame.extid);
        comm::can2_tp::ControlFlowID out;
        EXPECT_FALSE(can2tp.process_control_flow_frame(frame, &out));
    }

    TEST_F(CAN2TPTest, SetSendMessage_LengthTooSmall_Death) {
        uint8_t small[5] = {0};
        ASSERT_DEATH(can2tp.set_send_message(small, sizeof(small), 1, 2), ".*");
    }

    TEST_F(CAN2TPTest, GetNextSendFragment_NoMessageReturnsFalse) {
        comm::can2_tp::CAN2BFrame frame;
        EXPECT_FALSE(can2tp.get_next_send_fragment(frame));
    }

    TEST_F(CAN2TPTest, Segmented_LengthMultipleOf8) {
        const size_t msg_len = 16; // exactly multiple of 8
        auto message = BuildPayload(msg_len);
        comm::can2_tp::CAN2TP<MaxMessageSize> sender{CAN2TPTest::source};
        sender.set_send_message(message.data(), message.size(), 3, 2);
        auto frames = DrainFragments(sender);
        // FirstFrame + one or more consecutive frames; expect at least 2
        EXPECT_GE(frames.size(), 2u);
        auto reconstructed = ConcatPayloadsFromFrames(frames);
        EXPECT_EQ(reconstructed.size(), message.size());
        for (size_t i = 0; i < message.size(); ++i) EXPECT_EQ(reconstructed[i], message[i]);
    }

    TEST_F(CAN2TPTest, ProcessSegmentFrame_ConsecutiveBeforeFirst_ReturnsFalse) {
        const size_t msg_len = 20;
        auto message = BuildPayload(msg_len);
        comm::can2_tp::CAN2TP<MaxMessageSize> sender{CAN2TPTest::source};
        sender.set_send_message(message.data(), message.size(), 4, 2);
        auto frames = DrainFragments(sender);
        ASSERT_GT(frames.size(), 1u);

        comm::can2_tp::CAN2TP<MaxMessageSize> receiver{2};
        // Feed only the second frame (a ConsecutiveFrame) without a FirstFrame
        EXPECT_FALSE(receiver.process_segment_frame(frames[1]));
    }

    TEST_F(CAN2TPTest, GetReassembledMessage_NotComplete_ReturnsFalse) {
        const size_t msg_len = 20;
        auto message = BuildPayload(msg_len);
        comm::can2_tp::CAN2TP<MaxMessageSize> sender{CAN2TPTest::source};
        sender.set_send_message(message.data(), message.size(), 5, 2);
        auto frames = DrainFragments(sender);
        ASSERT_FALSE(frames.empty());

        comm::can2_tp::CAN2TP<MaxMessageSize> receiver{2};
        // Deliver only the first frame
        ASSERT_TRUE(receiver.process_segment_frame(frames[0]));

        uint8_t buf[MaxMessageSize] = {0};
        comm::protocol::TopicMessageMeta meta;
        EXPECT_FALSE(receiver.get_reassembled_message(buf, meta));
    }

}  // namespace