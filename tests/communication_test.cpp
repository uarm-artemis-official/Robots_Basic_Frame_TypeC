#include <gtest/gtest.h>
#include <algorithm>
#include <cstring>
#include <vector>
#include "fixtures/communication_submodule_fixture.hpp"

TEST_F(CommunicationSubmoduleFixture, InitSucceedsWithMockedIoAndFakeMc) {
    SUCCEED();
}

TEST_F(CommunicationSubmoduleFixture,
       CanIsrPendingPublishesDecodedDataToCorrectTopic) {
    using communication_fixture::TestDataMessage;

    EXPECT_CALL(rtos, get_current_tick()).WillRepeatedly(testing::Return(42));

    TestDataMessage input_message {.value = 0x1234};

    std::array<std::byte, TestDataMessage::SERIALIZED_SIZE>
        serialized_payload {};
    ASSERT_TRUE(
        TestDataMessage::serialize_payload(input_message, serialized_payload));

    simple_comm::SimpleMessage wire_message {};
    wire_message.message_type = simple_comm::MessageType::DATA;
    wire_message.source = static_cast<uint8_t>(simple_comm::NodeID::Chassis);
    wire_message.destination =
        static_cast<uint8_t>(simple_comm::NodeID::Gimbal);
    wire_message.id = TestDataMessage::MESSAGE_ID;
    wire_message.payload_size = TestDataMessage::SERIALIZED_SIZE;
    std::copy(serialized_payload.begin(), serialized_payload.end(),
              wire_message.payload.begin());

    MW_CAN::CANFrame received_frame {};
    simple_comm_codec.to_can_message<MW_CAN::CANFrame>(wire_message,
                                                       received_frame);

    communication.can_isr_message_pending(MW_CAN::BUS::CAN_2B, received_frame);

    Counters* counters = message_center.get_counters<TestDataMessage>();
    ASSERT_NE(counters, nullptr);
    EXPECT_EQ(counters->pub_count, 1u);
    ASSERT_EQ(counters->last_pub_bytes.size(), sizeof(TestDataMessage));

    TestDataMessage published_message {};
    std::memcpy(&published_message, counters->last_pub_bytes.data(),
                sizeof(TestDataMessage));

    EXPECT_EQ(published_message.value, input_message.value);
}

TEST_F(CommunicationSubmoduleFixture,
       CanIsrPendingForwardsToCanWithCorrectData) {
    using communication_fixture::TestDataMessage;

    TestDataMessage input_message {.value = 0xBEEF};

    std::array<std::byte, TestDataMessage::SERIALIZED_SIZE>
        serialized_payload {};
    ASSERT_TRUE(
        TestDataMessage::serialize_payload(input_message, serialized_payload));

    simple_comm::SimpleMessage incoming_message {};
    incoming_message.message_type = simple_comm::MessageType::DATA;
    incoming_message.source = static_cast<uint8_t>(simple_comm::NodeID::MiniPC);
    incoming_message.destination =
        static_cast<uint8_t>(simple_comm::NodeID::Chassis);
    incoming_message.id = TestDataMessage::MESSAGE_ID;
    incoming_message.payload_size = TestDataMessage::SERIALIZED_SIZE;
    std::copy(serialized_payload.begin(), serialized_payload.end(),
              incoming_message.payload.begin());

    MW_CAN::CANFrame incoming_frame {};
    simple_comm_codec.to_can_message<MW_CAN::CANFrame>(incoming_message,
                                                       incoming_frame);

    MW_CAN::BUS forwarded_bus = MW_CAN::BUS::CAN_1;
    uint32_t forwarded_sid = 0;
    uint32_t forwarded_eid = 0;
    uint32_t forwarded_length = 0;
    std::vector<uint8_t> forwarded_data;

    EXPECT_CALL(can, send_data(testing::_, testing::_, testing::_, testing::_,
                               testing::_))
        .WillOnce(testing::DoAll(
            testing::SaveArg<0>(&forwarded_bus),
            testing::SaveArg<1>(&forwarded_sid),
            testing::SaveArg<2>(&forwarded_eid),
            testing::SaveArg<4>(&forwarded_length),
            testing::WithArg<3>(testing::Invoke([&](const uint8_t* data) {
                forwarded_data.assign(data, data + incoming_frame.dlc);
            })),
            testing::Return(true)));

    communication.can_isr_message_pending(MW_CAN::BUS::CAN_2B, incoming_frame);

    MW_CAN::CANFrame expected_forwarded_frame {};
    simple_comm_codec.to_can_message<MW_CAN::CANFrame>(
        incoming_message, expected_forwarded_frame);

    EXPECT_EQ(forwarded_bus, MW_CAN::BUS::CAN_2B);
    EXPECT_EQ(forwarded_sid, expected_forwarded_frame.sid);
    EXPECT_EQ(forwarded_eid, expected_forwarded_frame.eid);
    EXPECT_EQ(forwarded_length, expected_forwarded_frame.dlc);
    ASSERT_EQ(forwarded_data.size(), expected_forwarded_frame.dlc);

    std::array<uint8_t, simple_comm::MAX_PAYLOAD_SIZE> expected_payload {};
    for (size_t i = 0; i < expected_forwarded_frame.dlc; ++i) {
        expected_payload[i] = static_cast<uint8_t>(
            std::to_integer<uint8_t>(expected_forwarded_frame.payload[i]));
    }
    EXPECT_TRUE(std::equal(forwarded_data.begin(), forwarded_data.end(),
                           expected_payload.begin()));
}