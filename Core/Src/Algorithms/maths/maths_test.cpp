#include <gtest/gtest.h>
#include "maths.h"


TEST(MathsTests, BasicAbsLimit) {
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

TEST(MathsTests, NegativeAbsLimit) {
    float num = 6.4f;
    abs_limit(&num, -10.0f);
    EXPECT_FLOAT_EQ(num, 6.4f) << "abs_limit is not supposed to limit.";

    num = 100.0f;
    abs_limit(&num, -5.0f);
    EXPECT_FLOAT_EQ(num, 100.0f) << "abs_limit is not supposed to limit.";

    num = -3.4f;
    abs_limit(&num, -10.0f);
    EXPECT_FLOAT_EQ(num, -3.4f) << "abs_limit is not supposed to limit.";

    num = -100.0f;
    abs_limit(&num, -5.0f);
    EXPECT_FLOAT_EQ(num, -100.0f) << "abs_limit is not supposed to limit.";
}