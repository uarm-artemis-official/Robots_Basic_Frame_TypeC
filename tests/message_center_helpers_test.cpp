#include "gtest/gtest.h"
#include "message_center.hpp"
#include "simple_comm_utils.hpp"
#include "topics.hpp"

namespace {

    TEST(MessageCenterHelpersBasic, GenerateInterboardTopicIds) {
        auto interboard_topic_ids =
            simple_comm::generate_interboard_message_ids<mc2::RobotTopics>();
        std::array<uint8_t, 4> expected_ids = {
            mc2::get_topic_id_from_index(3),  // GimbalCommand
            mc2::get_topic_id_from_index(4),  // ShootCommand
            mc2::get_topic_id_from_index(5),  // RefereeInfo
            mc2::get_topic_id_from_index(9),  // GimbalRelativeAngles
        };

        ASSERT_EQ(interboard_topic_ids.size(), expected_ids.size());
        for (size_t i = 0; i < expected_ids.size(); ++i)
            ASSERT_EQ(interboard_topic_ids[i], expected_ids[i]);
    }
}  // namespace
