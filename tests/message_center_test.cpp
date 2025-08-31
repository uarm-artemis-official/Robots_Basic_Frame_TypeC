#include "message_center.hpp"
#include <gtest/gtest.h>
#include <tuple>
#include "message_center.cpp"
#include "middleware_interfaces.hpp"
#include "middleware_types.hpp"
#include "subsystems_interfaces.h"
#include "subsystems_types.hpp"
#include "test_middleware.cpp"

struct FloatTopic : mc2::Topic<2> {
    float field;
};

struct ByteTopic : mc2::Topic<2> {
    uint8_t field;
};

struct WordTopic : mc2::Topic<2> {
    int32_t field;
};

using TopicRegistry = std::tuple<FloatTopic, ByteTopic, WordTopic>;

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

TEST_F(MessageCenterTest, PubAndGetMessageUint8) {
    ByteTopic value;
    value.field = 42;
    auto ts_pub = mc2->pub_message(value);
    ASSERT_TRUE(ts_pub.has_value());
    ASSERT_EQ(ts_pub.value(), start_ts);

    ByteTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, PubAndGetMessageInt32) {
    WordTopic value;
    value.field = -123456;
    auto ts_pub = mc2->pub_message(value);
    ASSERT_TRUE(ts_pub.has_value());
    ASSERT_EQ(ts_pub.value(), start_ts);

    WordTopic received;
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
    WordTopic value;
    value.field = 98765;
    bool context_switch = false;
    auto ts_pub = mc2->pub_message_from_isr(value);
    ASSERT_TRUE(ts_pub.has_value());
    ASSERT_EQ(ts_pub.value(), start_ts);

    WordTopic received;
    auto ts_get = mc2->get_message(received, 0);
    ASSERT_TRUE(ts_get.has_value());
    ASSERT_EQ(received.field, value.field);
    ASSERT_EQ(ts_pub.value(), ts_get.value());
}

TEST_F(MessageCenterTest, QueueOverflowReturnsZero) {
    // Assuming default queue_size > 1 for WordTopic
    WordTopic v1, v2, v3;
    v1.field = 1;
    v2.field = 2;
    v3.field = 3;
    mc2->pub_message(v1);
    mc2->pub_message(v2);
    mc2->pub_message(v3);

    WordTopic r1, r2, r3;
    mc2->get_message(r1, 0);
    mc2->get_message(r2, 0);
    auto ts_empty = mc2->get_message(r3, 0);
    ASSERT_FALSE(ts_empty.has_value());
}

TEST_F(MessageCenterTest, TimestampUpdatesCorrectly) {
    ByteTopic value;
    value.field = 99;
    auto ts1 = mc2->pub_message(value);
    rtos->delay(5);
    auto ts2 = mc2->pub_message(value);
    ASSERT_TRUE(ts1.has_value());
    ASSERT_TRUE(ts2.has_value());

    ByteTopic received;
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