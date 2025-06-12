#include "pid.h"
#include <gtest/gtest.h>

TEST(PidCorrectness, PidInitBasic) {
    PID_t pid;
    pid_param_init(&pid, 1000, 100.f, 10.f, 1.f, 2.f, 3.f);
    EXPECT_FLOAT_EQ(pid.kp, 1.f);
    EXPECT_FLOAT_EQ(pid.ki, 2.f);
    EXPECT_FLOAT_EQ(pid.kd, 3.f);
    EXPECT_FLOAT_EQ(pid.cur_val, 0.f);
    EXPECT_FLOAT_EQ(pid.target_val, 0.f);
    EXPECT_FLOAT_EQ(pid.err, 0.f);
    EXPECT_FLOAT_EQ(pid.last_err, 0.f);
    EXPECT_FLOAT_EQ(pid.llast_err, 0.f);

    EXPECT_FLOAT_EQ(pid.pout, 0.f);
    EXPECT_FLOAT_EQ(pid.iout, 0.f);
    EXPECT_FLOAT_EQ(pid.dout, 0.f);
    EXPECT_FLOAT_EQ(pid.max_out, 1000.f);
    EXPECT_FLOAT_EQ(pid.max_err, 10.f);
    EXPECT_FLOAT_EQ(pid.max_i_out, 100.f);
    EXPECT_FLOAT_EQ(pid.total_out, 0.f);
}

TEST(PidCorrectness, Pid2InitBasic) {
    PID2_t pid2;
    pid2_init(pid2, 0.f, 1.f, 2.f, 3.f, 4.f, 0.f, 100.f);
    EXPECT_FLOAT_EQ(pid2.k_p, 0.f);
    EXPECT_FLOAT_EQ(pid2.k_i, 1.f);
    EXPECT_FLOAT_EQ(pid2.k_d, 2.f);
    EXPECT_FLOAT_EQ(pid2.beta, 3.f);
    EXPECT_FLOAT_EQ(pid2.yeta, 4.f);

    EXPECT_FLOAT_EQ(pid2.plant_value, 0.f);
    EXPECT_FLOAT_EQ(pid2.setpoint, 0.f);

    EXPECT_FLOAT_EQ(pid2.p_error, 0.f);
    EXPECT_FLOAT_EQ(pid2.i_error, 0.f);
    EXPECT_FLOAT_EQ(pid2.d_error, 0.f);
    EXPECT_FLOAT_EQ(pid2.prev_d_error, 0.f);

    EXPECT_FLOAT_EQ(pid2.p_out, 0.f);
    EXPECT_FLOAT_EQ(pid2.i_out, 0.f);
    EXPECT_FLOAT_EQ(pid2.d_out, 0.f);

    EXPECT_FLOAT_EQ(pid2.max_out, 100.f);
    EXPECT_FLOAT_EQ(pid2.min_out, 0.f);

    EXPECT_FLOAT_EQ(pid2.prev_total_out, 0.f);
    EXPECT_FLOAT_EQ(pid2.total_out, 0.f);
}

TEST(PidFuzz, PidInitInvalidMaxes) {
    PID_t pid;
    EXPECT_DEATH(pid_param_init(&pid, -10.f, 100.f, 50.f, 1.f, 2.f, 3.f), "")
        << "max_out cannot be negative.";
    EXPECT_DEATH(pid_param_init(&pid, 10.f, -100.f, 50.f, 1.f, 2.f, 3.f), "")
        << "max_i_out cannot be negative.";
    EXPECT_DEATH(pid_param_init(&pid, 10.f, 100.f, -50.f, 1.f, 2.f, 3.f), "")
        << "max_err cannot be negative.";
}

TEST(PidFuzz, Pid2InitInvalidMaxes) {
    PID2_t pid2;
    EXPECT_DEATH(pid2_init(pid2, 1.f, 2.f, 3.f, 4.f, 5.f, 100.f, -10.f), "")
        << "min_out cannot be greater than max_out.";
}