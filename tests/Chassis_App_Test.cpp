#include <gtest/gtest.h>
#include "Chassis_App.h"

TEST(ChassisInit, ChassisDataReset) {
    Chassis_t chassis;
    chassis_reset_data(&chassis);
    EXPECT_EQ(chassis.vx, 0);
    EXPECT_EQ(chassis.vy, 0);
    EXPECT_EQ(chassis.vz, 1);
}