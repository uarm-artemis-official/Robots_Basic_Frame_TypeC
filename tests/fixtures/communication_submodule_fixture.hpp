#ifndef __COMMUNICATION_SUBMODULE_FIXTURE_HPP
#define __COMMUNICATION_SUBMODULE_FIXTURE_HPP

#include <gtest/gtest.h>
#include "../fakes/fake_message_center.hpp"
#include "../mocks/middleware_mocks.hpp"
#include "communication.hpp"

namespace communication_fixture {
    struct TestDataMessage {
        static constexpr uint8_t TOPIC_ID = 50;
        static constexpr uint8_t MESSAGE_ID = TOPIC_ID;
        static constexpr size_t QUEUE_SIZE = 1;
        static constexpr size_t SERIALIZED_SIZE = 2;
        static constexpr simple_comm::MessageType MESSAGE_TYPE =
            simple_comm::MessageType::DATA;

        uint16_t value;

        static bool serialize_payload(
            const TestDataMessage& msg,
            std::span<std::byte, SERIALIZED_SIZE> dst) {
            dst[0] = std::byte {static_cast<uint8_t>(msg.value & 0xFF)};
            dst[1] = std::byte {static_cast<uint8_t>((msg.value >> 8) & 0xFF)};
            return true;
        }

        static bool deserialize_payload(
            std::span<const std::byte, SERIALIZED_SIZE> src,
            TestDataMessage& msg) {
            msg.value = static_cast<uint16_t>(
                static_cast<uint16_t>(std::to_integer<uint8_t>(src[0])) |
                (static_cast<uint16_t>(std::to_integer<uint8_t>(src[1])) << 8));
            return true;
        }
    };
}  // namespace communication_fixture

class CommunicationSubmoduleFixture : public testing::Test {
   protected:
    using TopicRegistry =
        mc2::create_topic_registry_t<communication_fixture::TestDataMessage>;
    using MessageCenter = FakeMessageCenter<TopicRegistry>;
    using Communication = comm::Communication<MessageCenter, TopicRegistry>;

    MockIRTOS rtos;
    MockICAN can;
    MockIUART uart;
    MessageCenter message_center;
    simple_comm::SimpleCommCodec simple_comm_codec;
    Communication communication;

    CommunicationSubmoduleFixture()
        : message_center(rtos),
          communication(message_center, simple_comm_codec, can, uart) {
        communication.set_node_id(simple_comm::NodeID::Gimbal);
    }

    void SetUp() override {
        EXPECT_TRUE(communication.init());
        EXPECT_TRUE(message_center.init());
    }
};

#endif