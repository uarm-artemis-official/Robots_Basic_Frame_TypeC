#include "gtest/gtest.h"
#include "multiboard_fixture.hpp"

TEST_F(MultiboardCAN2Test, BasicTest) {
    EXPECT_CALL(*rtos, get_current_tick)
        .Times(testing::AtLeast(1))
        .WillRepeatedly(testing::Return(5));

    mc2::GimbalCommand gimbal_command;
    gimbal_command.yaw = 1.0f;
    gimbal_command.pitch = 2.0f;
    gimbal_command.command_bits = 0;
    message_center->set_get_return_value<mc2::GimbalCommand>(gimbal_command);

    EXPECT_EQ(message_center->get_counters<mc2::GimbalCommand>()->pub_count, 0);

    mc2::GimbalCommand new_gimbal_command;
    new_gimbal_command.yaw = 0.5f;
    new_gimbal_command.pitch = 0.5f;
    new_gimbal_command.command_bits = 10;
    message_center->pub_message(new_gimbal_command);

    EXPECT_GT(message_center->get_counters<mc2::GimbalCommand>()->pub_count, 0);
}