#include <gtest/gtest.h>
#include <tuple>
#include "middleware_classes.hpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "subsystems_classes.hpp"
#include "subsystems_interfaces.hpp"
#include "subsystems_types.hpp"

struct FloatTopic : mc2::Topic<2> {
    float field;
};

struct StructTopic : mc2::Topic<5> {
    uint8_t byte;
    float f;
    int16_t hw;
    uint32_t w;
};

struct MailboxTopic : mc2::Topic<1> {
    uint8_t field;
};

using TopicRegistry = std::tuple<FloatTopic, StructTopic, MailboxTopic>;

class MessageCenterTest : public ::testing::Test {
   protected:
    MW_RTOS::TestRTOS* rtos;
    mc2::MC2<TopicRegistry>* mc2;
    MW_RTOS::TickType start_ts = 5;

    void SetUp() override {
        rtos = new MW_RTOS::TestRTOS();
        mc2 = new mc2::MC2<TopicRegistry>(*rtos);
        mc2->init();
        rtos->delay(start_ts);
    }
};

TEST_F(MessageCenterTest, PubAndGetMessageFloat) {
    FloatTopic value;
    value.field = 3.14f;
    auto ts_pub = mc2->pub_message(value);
    ASSERT_TRUE(ts_pub.has_value());

    FloatTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PeekMessageDoesNotRemove) {
    FloatTopic value;
    value.field = 1.23f;
    mc2->pub_message(value);

    FloatTopic peeked;
    auto ts_peek = mc2->peek_message(peeked, 0);
    ASSERT_TRUE(ts_peek.has_value());
    ASSERT_EQ(peeked.field, value.field);

    FloatTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_peek.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PubMessageFromISRWorks) {
    FloatTopic value;
    value.field = 987.65f;
    auto ts_pub = mc2->pub_message_from_isr(value);
    ASSERT_TRUE(ts_pub.has_value());
    ASSERT_EQ(ts_pub.value(), start_ts);

    FloatTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, QueueOverflowReturnsZero) {
    // Assuming default queue_size > 1 for FloatTopic
    FloatTopic v1, v2, v3;
    v1.field = 1.0f;
    v2.field = 2.0f;
    v3.field = 3.0f;
    mc2->pub_message(v1);
    mc2->pub_message(v2);
    mc2->pub_message(v3);

    FloatTopic r1, r2, r3;
    mc2->get_message(r1, 0);
    mc2->get_message(r2, 0);
    auto ts_empty = mc2->get_message(r3, 0);
    ASSERT_FALSE(ts_empty.has_value());
}

TEST_F(MessageCenterTest, TimestampUpdatesCorrectly) {
    FloatTopic value;
    value.field = 99.0f;
    auto ts1 = mc2->pub_message(value);
    rtos->delay(5);
    auto ts2 = mc2->pub_message(value);
    ASSERT_TRUE(ts1.has_value());
    ASSERT_TRUE(ts2.has_value());

    FloatTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(ts_get.value(), ts2.value());

    auto ts_get2 = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get2.has_value());
    ASSERT_EQ(ts_get2.value(), ts1.value());
    ASSERT_EQ(ts_get2.value(), start_ts);
}

TEST_F(MessageCenterTest, GetMessageReturnsZeroIfEmpty) {
    FloatTopic received;
    auto ts = mc2->get_message(received, 0);
    ASSERT_FALSE(ts.has_value());
}

TEST_F(MessageCenterTest, PubAndGetMessageStructTopic) {
    StructTopic value;
    value.byte = 0xAB;
    value.f = 2.71f;
    value.hw = -1234;
    value.w = 0xDEADBEEF;
    auto ts_pub = mc2->pub_message(value);
    ASSERT_TRUE(ts_pub.has_value());

    StructTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.byte, value.byte);
    ASSERT_EQ(received.f, value.f);
    ASSERT_EQ(received.hw, value.hw);
    ASSERT_EQ(received.w, value.w);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PeekMessageDoesNotRemoveStructTopic) {
    StructTopic value;
    value.byte = 0x55;
    value.f = 1.23f;
    value.hw = 4321;
    value.w = 0x12345678;
    mc2->pub_message(value);

    StructTopic peeked;
    auto ts_peek = mc2->peek_message(peeked, 0);
    ASSERT_TRUE(ts_peek.has_value());
    ASSERT_EQ(peeked.byte, value.byte);
    ASSERT_EQ(peeked.f, value.f);
    ASSERT_EQ(peeked.hw, value.hw);
    ASSERT_EQ(peeked.w, value.w);

    StructTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.byte, value.byte);
    ASSERT_EQ(received.f, value.f);
    ASSERT_EQ(received.hw, value.hw);
    ASSERT_EQ(received.w, value.w);
    ASSERT_EQ(ts_peek.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PubAndGetMessageMailboxTopic) {
    MailboxTopic value;
    value.field = 0x42;
    auto ts_pub = mc2->pub_message(value);
    ASSERT_TRUE(ts_pub.has_value());

    MailboxTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PeekMessageDoesNotRemoveMailboxTopic) {
    MailboxTopic value;
    value.field = 0x99;
    mc2->pub_message(value);

    MailboxTopic peeked;
    auto ts_peek = mc2->peek_message(peeked, 0);
    ASSERT_TRUE(ts_peek.has_value());
    ASSERT_EQ(peeked.field, value.field);

    MailboxTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_peek.value(), ts_get.value());
}

TEST_F(MessageCenterTest, MailboxTopicOverwriteBehavior) {
    MailboxTopic first;
    first.field = 0x11;
    MailboxTopic second;
    second.field = 0x22;

    // Publish first message
    auto ts_pub1 = mc2->pub_message(first);
    ASSERT_TRUE(ts_pub1.has_value());

    // Publish second message, should overwrite the first
    auto ts_pub2 = mc2->pub_message(second);
    ASSERT_TRUE(ts_pub2.has_value());

    // Get should retrieve the second (latest) message
    MailboxTopic received;
    auto ts_get1 = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get1.has_value());
    ASSERT_EQ(received.field, second.field);
    ASSERT_EQ(ts_get1.value(), ts_pub2.value());

    // Second get should fail (mailbox is now empty)
    MailboxTopic received2;
    auto ts_get2 = mc2->get_message(received2, 0);
    ASSERT_FALSE(ts_get2.has_value());
}

TEST_F(MessageCenterTest, PeekEmptyTopicFails) {
    FloatTopic peeked;
    auto ts_peek = mc2->peek_message(peeked, 0);
    ASSERT_FALSE(ts_peek.has_value());
}

TEST_F(MessageCenterTest, StructTopicMultiplePubAndGetSequence) {
    // Fill the queue with 5 messages (StructTopic queue size is 5)
    StructTopic s1 {{}, 0x01, 1.1f, 11, 0x11111111};
    StructTopic s2 {{}, 0x02, 2.2f, 22, 0x22222222};
    StructTopic s3 {{}, 0x03, 3.3f, 33, 0x33333333};
    StructTopic s4 {{}, 0x04, 4.4f, 44, 0x44444444};
    StructTopic s5 {{}, 0x05, 5.5f, 55, 0x55555555};

    ASSERT_TRUE(mc2->pub_message(s1).has_value());
    ASSERT_TRUE(mc2->pub_message(s2).has_value());
    ASSERT_TRUE(mc2->pub_message(s3).has_value());

    // Get one message (should be s1)
    StructTopic got;
    auto ts1 = mc2->get_message(got, 0);
    ASSERT_TRUE(ts1.has_value());
    ASSERT_EQ(got.byte, s1.byte);
    ASSERT_EQ(got.f, s1.f);
    ASSERT_EQ(got.hw, s1.hw);
    ASSERT_EQ(got.w, s1.w);

    // Add two more (s4, s5)
    ASSERT_TRUE(mc2->pub_message(s4).has_value());
    ASSERT_TRUE(mc2->pub_message(s5).has_value());

    // Now queue should have s2, s3, s4, s5 (in order)
    StructTopic s6 {{}, 0x06, 6.6f, 66, 0x66666666};
    StructTopic s7 {{}, 0x07, 7.7f, 77, 0x77777777};
    ASSERT_TRUE(mc2->pub_message(s6).has_value());
    ASSERT_FALSE(mc2->pub_message(s7).has_value());

    // Now queue should have s2, s3, s4, s5, s6 (in order)
    StructTopic expected[] = {s2, s3, s4, s5, s6};
    for (const auto& exp : expected) {
        StructTopic got2;
        auto ts = mc2->get_message(got2, 0);
        ASSERT_TRUE(ts.has_value());
        ASSERT_EQ(got2.byte, exp.byte);
        ASSERT_EQ(got2.f, exp.f);
        ASSERT_EQ(got2.hw, exp.hw);
        ASSERT_EQ(got2.w, exp.w);
    }

    // Queue should now be empty
    StructTopic empty;
    auto ts_empty = mc2->get_message(empty, 0);
    ASSERT_FALSE(ts_empty.has_value());
}