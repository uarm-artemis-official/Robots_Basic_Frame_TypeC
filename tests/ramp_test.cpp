#include "ramp.hpp"
#include <gtest/gtest.h>

TEST(RampCorrectness, RampInitBasic) {
    Ramp ramp;
    ramp_init(ramp, 5);
    EXPECT_FLOAT_EQ(ramp.target, 0.f);
    EXPECT_FLOAT_EQ(ramp.start, 0.f);
    EXPECT_FLOAT_EQ(ramp.max_change, 5.f);
    EXPECT_FLOAT_EQ(ramp.output, 0.f);
    EXPECT_FLOAT_EQ(ramp.cumsum_dt, 0.f);
}

TEST(RampCorrectness, RampCalcIdempotent) {
    Ramp ramp;
    ramp_init(ramp, 10);
    ramp_set_target(ramp, 10, 10);
    ramp_calc_output(ramp, 1.f);
    EXPECT_FLOAT_EQ(ramp.output, 10);
}

TEST(RampCorrectness, RampCalcBasicFunc) {
    Ramp ramp;
    ramp_init(ramp, 0);
    ramp_set_target(ramp, 0, 10);
    ramp_calc_output(ramp, 1.f);
    EXPECT_FLOAT_EQ(ramp.output, 0);

    Ramp ramp2;
    ramp_init(ramp2, 0);
    ramp_set_target(ramp2, 10, 0);
    ramp_calc_output(ramp2, 1.f);
    EXPECT_FLOAT_EQ(ramp2.output, 10);

    Ramp ramp3;
    ramp_init(ramp3, 1);
    ramp_set_target(ramp3, 10, 0);
    ramp_calc_output(ramp3, 1.f);
    EXPECT_FLOAT_EQ(ramp3.output, 9);

    Ramp ramp4;
    ramp_init(ramp4, 1);
    ramp_set_target(ramp4, 0, 10);
    ramp_calc_output(ramp4, 1.f);
    EXPECT_FLOAT_EQ(ramp4.output, 1);
}

TEST(RampCorrectness, RampInitInvalidMax) {
    Ramp ramp;
    EXPECT_DEATH(ramp_init(ramp, -1), "")
        << "Ramp max change cannot be negative.";
}