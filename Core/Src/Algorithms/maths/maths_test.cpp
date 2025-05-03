#include <gtest/gtest.h>
#include "maths.h"


TEST(MathsCorrectness, BasicAbsLimit) {
    float num = 6.4f;
    abs_limit(&num, 10.0f);
    EXPECT_FLOAT_EQ(num, 6.4f) << "abs_limit is not supposed to limit.";

    num = 100.0f;
    abs_limit(&num, 5.0f);
    EXPECT_FLOAT_EQ(num, 5.0f) << "abs_limit is supposed to limit.";

    num = -3.4f;
    abs_limit(&num, 10.0f);
    EXPECT_FLOAT_EQ(num, -3.4f) << "abs_limit is not supposed to limit.";

    num = -100.0f;
    abs_limit(&num, 5.0f);
    EXPECT_FLOAT_EQ(num, -5.0f) << "abs_limit is supposed to limit.";
}

TEST(MathsCorrectness, BasicInOutMap) {
    EXPECT_FLOAT_EQ(in_out_map(10.0f, 0.f, 20.f, 0.f, 100.f), 50.f);
    EXPECT_FLOAT_EQ(in_out_map(0.0f, 0.f, 20.f, 0.f, 100.f), 0.f);
    EXPECT_FLOAT_EQ(in_out_map(20.0f, 0.f, 20.f, 0.f, 100.f), 100.f);
}

TEST(MathsFuzz, NegativeAbsLimit) {
    float num = 100.0f;
    EXPECT_DEATH(abs_limit(&num, -10.0f), "") << "abs_limit should not accept negative limits.";
}

TEST(MathsFuzz, InvalidInputInOutMap) {
    EXPECT_DEATH(in_out_map(10.f, 0.f, 5.f, 0.f, 100.f), "") << "Inputs larger than in_max are unacceptable.";
    EXPECT_DEATH(in_out_map(-10.f, 0.f, 5.f, 0.f, 100.f), "") << "Inputs smaller than in_min are unacceptable.";
    EXPECT_DEATH(in_out_map(2.f, 10.f, 0.f, 0.f, 100.f), "") << "Flipped parameters for input domain.";
    EXPECT_DEATH(in_out_map(2.f, 0.f, 10.f, 50.f, 0.f), "") << "Flipped parameters for output domain.";
}

