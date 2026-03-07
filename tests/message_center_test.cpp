#include "message_center.hpp"
#include <gtest/gtest.h>
#include <tuple>
#include "simple_comm_utils.hpp"
#include "middleware_classes.hpp"

// Additional test topics
struct ExtraNormalTopic {
    static constexpr size_t queue_size = 3;
    int32_t i;
    float f;
    char c;
};

struct InterTopicA {
    static constexpr mc2::MessageNode destination = mc2::MessageNode::Chassis;
    static constexpr size_t serialized_size = 3;  // 1 + 2 bytes
    static constexpr size_t queue_size = 2;

    uint8_t a;
    uint16_t b;

    static bool serialize(const InterTopicA& msg,
                          std::span<std::byte, serialized_size> dst) {
        dst[0] = std::byte {static_cast<uint8_t>(msg.a)};
        dst[1] = std::byte {static_cast<uint8_t>(msg.b & 0xFF)};
        dst[2] = std::byte {static_cast<uint8_t>((msg.b >> 8) & 0xFF)};
        return true;
    }

    static bool deserialize(InterTopicA& msg,
                            std::span<const std::byte, serialized_size> src) {
        msg.a = std::to_integer<uint8_t>(src[0]);
        msg.b = static_cast<uint16_t>(std::to_integer<uint8_t>(src[1])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[2])) << 8);
        return true;
    }
};

struct InterTopicB {
    static constexpr mc2::MessageNode destination = mc2::MessageNode::Gimbal;
    static constexpr size_t serialized_size = 8;  // 8 bytes
    static constexpr size_t queue_size = 1;

    uint64_t v;

    static bool serialize(const InterTopicB& msg,
                          std::span<std::byte, serialized_size> dst) {
        uint64_t val = msg.v;
        for (size_t i = 0; i < serialized_size; ++i) {
            dst[i] = std::byte {static_cast<uint8_t>((val >> (8 * i)) & 0xFF)};
        }
        return true;
    }

    static bool deserialize(InterTopicB& msg,
                            std::span<const std::byte, serialized_size> src) {
        uint64_t val = 0;
        for (size_t i = 0; i < serialized_size; ++i) {
            val |= static_cast<uint64_t>(std::to_integer<uint8_t>(src[i]))
                   << (8 * i);
        }
        msg.v = val;
        return true;
    }
};

struct FloatTopic {
    static constexpr size_t queue_size = 2;

    float field;
};

struct StructTopic {
    static constexpr size_t queue_size = 5;

    uint8_t byte;
    float f;
    int16_t hw;
    uint32_t w;
};

struct MailboxTopic {
    static constexpr size_t queue_size = 1;
    uint8_t field;
};

using TopicRegistry = std::tuple<FloatTopic, StructTopic, MailboxTopic>;
using ExtendedTopicRegistry =
    std::tuple<FloatTopic, StructTopic, MailboxTopic, ExtraNormalTopic,
               InterTopicA, InterTopicB>;

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

    void TearDown() override {
        delete mc2;
        delete rtos;
    }
};

class ExtendedMessageCenterTest : public ::testing::Test {
   protected:
    MW_RTOS::TestRTOS* rtos;
    mc2::MC2<ExtendedTopicRegistry>* mc;

    void SetUp() override {
        rtos = new MW_RTOS::TestRTOS();
        mc = new mc2::MC2<ExtendedTopicRegistry>(*rtos);
        mc->init();
    }

    void TearDown() override {
        delete mc;
        delete rtos;
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
    StructTopic s1 {0x01, 1.1f, 11, 0x11111111};
    StructTopic s2 {0x02, 2.2f, 22, 0x22222222};
    StructTopic s3 {0x03, 3.3f, 33, 0x33333333};
    StructTopic s4 {0x04, 4.4f, 44, 0x44444444};
    StructTopic s5 {0x05, 5.5f, 55, 0x55555555};

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
    StructTopic s6 {0x06, 6.6f, 66, 0x66666666};
    StructTopic s7 {0x07, 7.7f, 77, 0x77777777};
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

TEST_F(ExtendedMessageCenterTest, SerializeByTopicIDHappyPath) {
    InterTopicA orig {};
    orig.a = 0x5;
    orig.b = 0x1234;

    std::array<std::byte, sizeof(InterTopicA)> src_bytes {};
    std::memcpy(src_bytes.data(), &orig, sizeof(orig));

    std::array<std::byte, InterTopicA::serialized_size> serialized_by_method {};
    std::array<std::byte, InterTopicA::serialized_size> serialized_by_type {};

    uint8_t topic_id = static_cast<uint8_t>(
        mc2::get_comm_id<InterTopicA, ExtendedTopicRegistry>());
    auto serializers = simple_comm::utils::generate_message_serializers<ExtendedTopicRegistry>();
    bool s_ok = serializers[mc2::get_index_from_topic_id(topic_id)](
        std::span(serialized_by_method), std::span(src_bytes));
    ASSERT_TRUE(s_ok);

    bool s_ok_type = InterTopicA::serialize(
        orig,
        std::span<std::byte, InterTopicA::serialized_size>(serialized_by_type));

    ASSERT_TRUE(s_ok_type);

    for (size_t i = 0; i < InterTopicA::serialized_size; ++i) {
        ASSERT_EQ(serialized_by_method[i], serialized_by_type[i]);
    }
}

TEST_F(ExtendedMessageCenterTest, DeserializeByTopicIDHappyPath) {
    InterTopicB orig {};
    orig.v = 0x1122334455667788;

    std::array<std::byte, InterTopicB::serialized_size> src_bytes {};
    InterTopicB::serialize(
        orig, std::span<std::byte, InterTopicB::serialized_size>(src_bytes));

    std::array<std::byte, sizeof(InterTopicB)> deserialized_by_method {};
    std::array<std::byte, sizeof(InterTopicB)> deserialized_by_type {};

    uint8_t topic_id = static_cast<uint8_t>(
        mc2::get_comm_id<InterTopicB, ExtendedTopicRegistry>());
    auto deserializers = simple_comm::utils::generate_message_deserializers<ExtendedTopicRegistry>();
    bool d_ok = deserializers[mc2::get_index_from_topic_id(topic_id)](
        std::span(deserialized_by_method), std::span(src_bytes));
    ASSERT_TRUE(d_ok);

    InterTopicB temp;
    bool d_ok_type = InterTopicB::deserialize(
        temp,
        std::span<const std::byte, InterTopicB::serialized_size>(src_bytes));
    ASSERT_TRUE(d_ok_type);
    std::memcpy(deserialized_by_type.data(), &temp, sizeof(InterTopicB));

    for (size_t i = 0; i < sizeof(InterTopicB); ++i) {
        ASSERT_EQ(deserialized_by_method[i], deserialized_by_type[i]);
    }
}

TEST_F(ExtendedMessageCenterTest, SerializeDeserializeByTopicIDHappyPath) {
    InterTopicA orig {};
    orig.a = 0x9;
    orig.b = 0xBEEF;

    std::array<std::byte, sizeof(InterTopicA)> src_bytes {};
    std::memcpy(src_bytes.data(), &orig, sizeof(orig));

    std::array<std::byte, InterTopicA::serialized_size> serialized {};
    std::array<std::byte, sizeof(InterTopicA)> deserialized {};

    uint8_t topic_id = static_cast<uint8_t>(
        mc2::get_comm_id<InterTopicA, ExtendedTopicRegistry>());
    auto serializers2 = simple_comm::utils::generate_message_serializers<ExtendedTopicRegistry>();
    bool s_ok = serializers2[mc2::get_index_from_topic_id(topic_id)](
        std::span(serialized), std::span(src_bytes));
    ASSERT_TRUE(s_ok);

    auto deserializers2 = simple_comm::utils::generate_message_deserializers<ExtendedTopicRegistry>();
    bool d_ok = deserializers2[mc2::get_index_from_topic_id(topic_id)](
        std::span(deserialized), std::span(serialized));
    ASSERT_TRUE(d_ok);

    for (size_t i = 0; i < sizeof(InterTopicA); ++i) {
        ASSERT_EQ(deserialized[i], src_bytes[i]);
    }
}

TEST_F(ExtendedMessageCenterTest, Serialize_Fail_NonInterboardTopic) {
    ExtraNormalTopic t {};
    t.i = 1;
    t.f = 1.5f;
    t.c = 'x';

    std::array<std::byte, sizeof(ExtraNormalTopic)> src_bytes {};
    std::memcpy(src_bytes.data(), &t, sizeof(t));

    std::array<std::byte, 4> dst {};  // arbitrary

    uint8_t topic_id = static_cast<uint8_t>(
        mc2::get_comm_id<ExtraNormalTopic, ExtendedTopicRegistry>());
    auto serializers3 = simple_comm::utils::generate_message_serializers<ExtendedTopicRegistry>();
    bool s_ok = serializers3[mc2::get_index_from_topic_id(topic_id)](
        std::span(dst), std::span(src_bytes));
    ASSERT_FALSE(s_ok);

    auto deserializers3 = simple_comm::utils::generate_message_deserializers<ExtendedTopicRegistry>();
    bool d_ok = deserializers3[mc2::get_index_from_topic_id(topic_id)](
        std::span(dst), std::span(src_bytes));
    ASSERT_FALSE(d_ok);
}