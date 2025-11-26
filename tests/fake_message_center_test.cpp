#include "fake_message_center.hpp"
#include <gtest/gtest.h>
#include "message_center.hpp"
#include "middleware_mocks.hpp"

// Define simple test message types
struct TestMessage1 {
    int32_t value;
    float temperature;

    bool operator==(const TestMessage1& other) const {
        return value == other.value && temperature == other.temperature;
    }
};

struct TestMessage2 {
    uint16_t id;
    uint8_t status;

    bool operator==(const TestMessage2& other) const {
        return id == other.id && status == other.status;
    }
};

// Define test topic registry
using TestTopicRegistry = std::tuple<TestMessage1, TestMessage2>;

class FakeMessageCenterTest : public ::testing::Test {
   protected:
    MockIRTOS mock_rtos;
    FakeMessageCenter<TestTopicRegistry> fake_mc {mock_rtos};

    void SetUp() override {
        // Set mock return values before any message center operations
        EXPECT_CALL(mock_rtos, get_current_tick())
            .WillRepeatedly(::testing::Return(12345));

        EXPECT_TRUE(fake_mc.init());
    }
};

// Test: Set and get return value for a topic
TEST_F(FakeMessageCenterTest, SetGetReturnValueBasic) {
    TestMessage1 original {.value = 42, .temperature = 23.5f};
    fake_mc.set_get_return_value<TestMessage1>(original);

    TestMessage1 retrieved {};
    auto result = fake_mc.get_message<TestMessage1>(retrieved);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(retrieved, original);
}

// Test: Get counter increments when get_message is called
TEST_F(FakeMessageCenterTest, GetCounterIncrementsOnGetMessage) {
    TestMessage1 msg {.value = 100, .temperature = 20.0f};
    fake_mc.set_get_return_value<TestMessage1>(msg);

    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);
    EXPECT_EQ(counters->get_count, 0);

    TestMessage1 retrieved {};
    fake_mc.get_message<TestMessage1>(retrieved);
    EXPECT_EQ(counters->get_count, 1);

    fake_mc.get_message<TestMessage1>(retrieved);
    EXPECT_EQ(counters->get_count, 2);
}

// Test: Pub counter increments when pub_message is called
TEST_F(FakeMessageCenterTest, PubCounterIncrementsOnPubMessage) {
    TestMessage1 msg {.value = 50, .temperature = 25.0f};

    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);
    EXPECT_EQ(counters->pub_count, 0);

    TestMessage1 to_publish = msg;
    fake_mc.pub_message<TestMessage1>(to_publish);
    EXPECT_EQ(counters->pub_count, 1);

    to_publish.value = 75;
    fake_mc.pub_message<TestMessage1>(to_publish);
    EXPECT_EQ(counters->pub_count, 2);
}

// Test: Last published bytes are stored correctly
TEST_F(FakeMessageCenterTest, LastPubBytesStoredCorrectly) {
    TestMessage1 msg1 {.value = 111, .temperature = 11.5f};
    TestMessage1 msg2 {.value = 222, .temperature = 22.5f};

    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);

    TestMessage1 to_publish = msg1;
    fake_mc.pub_message<TestMessage1>(to_publish);

    EXPECT_EQ(counters->last_pub_bytes.size(), sizeof(TestMessage1));
    TestMessage1 retrieved1 {};
    std::memcpy(&retrieved1, counters->last_pub_bytes.data(),
                sizeof(TestMessage1));
    EXPECT_EQ(retrieved1, msg1);

    to_publish = msg2;
    fake_mc.pub_message<TestMessage1>(to_publish);

    TestMessage1 retrieved2 {};
    std::memcpy(&retrieved2, counters->last_pub_bytes.data(),
                sizeof(TestMessage1));
    EXPECT_EQ(retrieved2, msg2);
}

// Test: Multiple pub_messages update last_pub_bytes correctly
TEST_F(FakeMessageCenterTest, MultiplePubMessagesUpdateLastPubBytes) {
    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);

    for (int i = 0; i < 5; ++i) {
        TestMessage1 msg {.value = i * 10,
                          .temperature = static_cast<float>(i)};
        fake_mc.pub_message<TestMessage1>(msg);

        TestMessage1 retrieved {};
        std::memcpy(&retrieved, counters->last_pub_bytes.data(),
                    sizeof(TestMessage1));
        EXPECT_EQ(retrieved.value, i * 10);
        EXPECT_EQ(retrieved.temperature, static_cast<float>(i));
    }

    EXPECT_EQ(counters->pub_count, 5);
}

// Test: Get message returns nullopt when no return value is set
TEST_F(FakeMessageCenterTest, GetMessageReturnsNulloptWhenNoReturnValueSet) {
    TestMessage2 msg {};
    auto result = fake_mc.get_message<TestMessage2>(msg);
    EXPECT_FALSE(result.has_value());
}

// Test: Independent counters for different message types
TEST_F(FakeMessageCenterTest, IndependentCountersForDifferentTypes) {
    TestMessage1 msg1 {.value = 1, .temperature = 1.0f};
    TestMessage2 msg2 {.id = 100, .status = 1};

    fake_mc.set_get_return_value<TestMessage1>(msg1);
    fake_mc.set_get_return_value<TestMessage2>(msg2);

    auto* counters1 = fake_mc.get_counters<TestMessage1>();
    auto* counters2 = fake_mc.get_counters<TestMessage2>();

    TestMessage1 retrieved1 {};
    TestMessage2 retrieved2 {};

    fake_mc.get_message<TestMessage1>(retrieved1);
    fake_mc.get_message<TestMessage1>(retrieved1);
    fake_mc.get_message<TestMessage2>(retrieved2);

    EXPECT_EQ(counters1->get_count, 2);
    EXPECT_EQ(counters2->get_count, 1);
}

// Test: pub_message_from_isr increments counter
TEST_F(FakeMessageCenterTest, PubMessageFromISRIncrementsCounter) {
    TestMessage1 msg {.value = 77, .temperature = 77.7f};
    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);

    TestMessage1 to_publish = msg;
    bool will_switch = true;
    fake_mc.pub_message_from_isr<TestMessage1>(to_publish, &will_switch);

    EXPECT_EQ(counters->pub_count, 1);
    EXPECT_FALSE(will_switch);
}

// Test: peek_message increments get_count and returns value
TEST_F(FakeMessageCenterTest, PeekMessageIncrementsGetCount) {
    TestMessage1 msg {.value = 88, .temperature = 88.8f};
    fake_mc.set_get_return_value<TestMessage1>(msg);

    auto* counters = fake_mc.get_counters<TestMessage1>();
    ASSERT_NE(counters, nullptr);

    TestMessage1 retrieved {};
    auto result = fake_mc.peek_message<TestMessage1>(retrieved);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(retrieved, msg);
    EXPECT_EQ(counters->get_count, 1);
}

// Test: Setting return value overwrites previous value
TEST_F(FakeMessageCenterTest, SetReturnValueOverwritesPrevious) {
    TestMessage1 msg1 {.value = 111, .temperature = 11.1f};
    TestMessage1 msg2 {.value = 222, .temperature = 22.2f};

    fake_mc.set_get_return_value<TestMessage1>(msg1);
    TestMessage1 retrieved {};
    fake_mc.get_message<TestMessage1>(retrieved);
    EXPECT_EQ(retrieved, msg1);

    fake_mc.set_get_return_value<TestMessage1>(msg2);
    fake_mc.get_message<TestMessage1>(retrieved);
    EXPECT_EQ(retrieved, msg2);
}

// Test: Counters for all topics start at zero
TEST_F(FakeMessageCenterTest, AllCountersStartAtZero) {
    auto* counters1 = fake_mc.get_counters<TestMessage1>();
    auto* counters2 = fake_mc.get_counters<TestMessage2>();

    ASSERT_NE(counters1, nullptr);
    ASSERT_NE(counters2, nullptr);
    EXPECT_EQ(counters1->get_count, 0);
    EXPECT_EQ(counters1->pub_count, 0);
    EXPECT_EQ(counters2->get_count, 0);
    EXPECT_EQ(counters2->pub_count, 0);
}

// Test: Get message returns current tick
TEST_F(FakeMessageCenterTest, GetMessageReturnsTick) {
    TestMessage1 msg {.value = 42, .temperature = 42.0f};
    fake_mc.set_get_return_value<TestMessage1>(msg);

    TestMessage1 retrieved {};
    auto result = fake_mc.get_message<TestMessage1>(retrieved);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 12345);
}