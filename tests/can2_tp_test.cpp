#include "can2_tp.hpp"
#include <gtest/gtest.h>

namespace {
    class CAN2TPTest : public ::testing::Test {
       protected:
        static constexpr size_t MaxMessageSize = 64;
        static constexpr uint8_t source = 1;
        comm::can2_tp::CAN2TP<MaxMessageSize> can2tp {source};  // source = 1
    };

    TEST_F(CAN2TPTest, IsSendingMessageInitiallyFalse) {
        EXPECT_FALSE(can2tp.is_sending_message());
    }

    TEST_F(CAN2TPTest, IsSendingMessageTrueAfterSetSendMessage) {
        EXPECT_FALSE(can2tp.is_sending_message());

        uint8_t message[20] = {0};
        can2tp.set_send_message(message, sizeof(message), 1, 2);
        EXPECT_TRUE(can2tp.is_sending_message());
    }

    TEST_F(CAN2TPTest, GetSingleFrameValid) {
        uint8_t message[5] = {1, 2, 3, 4, 5};
        comm::can2_tp::CAN2BFrame frame;

        const uint8_t destination = 2;
        const uint8_t message_id = 10;

        bool result = can2tp.get_single_frame(message, sizeof(message),
                                              destination, frame);

        EXPECT_TRUE(result);
        EXPECT_EQ(frame.length, sizeof(message));
        EXPECT_EQ(comm::can2_tp::get_frame_type(frame.stdid),
                  comm::protocol::FrameType::SingleFrame);
        EXPECT_EQ(comm::can2_tp::get_source(frame.stdid), CAN2TPTest::source);
        EXPECT_EQ(comm::can2_tp::get_destination(frame.stdid), destination);
        EXPECT_EQ(comm::can2_tp::get_length(frame.extid), sizeof(message));
        EXPECT_EQ(comm::can2_tp::get_message_id(frame.extid), message_id);
        EXPECT_EQ(frame.payload[0], 1);
        EXPECT_EQ(frame.payload[1], 2);
        EXPECT_EQ(frame.payload[2], 3);
        EXPECT_EQ(frame.payload[3], 4);
        EXPECT_EQ(frame.payload[4], 5);
    }

}  // namespace