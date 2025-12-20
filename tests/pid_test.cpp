#include "pid.h"
#include <gtest/gtest.h>

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

TEST(PidCorrectness, Pid2PFuncBasic) {
    PID2_t pid;
    pid2_init(pid, 5, 0, 0, 1, 1, -1000, 1000);
    pid2_calculate(pid, 100, 0, 1);
    EXPECT_FLOAT_EQ(pid.total_out, 500);

    pid2_calculate(pid, -100, 0, 1);
    EXPECT_FLOAT_EQ(pid.total_out, -500);

    pid2_calculate(pid, 500, 0, 1);
    EXPECT_FLOAT_EQ(pid.total_out, 1000);

    pid2_calculate(pid, -500, 0, 1);
    EXPECT_FLOAT_EQ(pid.total_out, -1000);
}

TEST(PidCorrectness, Pid2SetLimitsValid) {
    PID2_t pid2;
    pid2_init(pid2, 1.f, 2.f, 3.f, 4.f, 5.f, 0.f, 100.f);
    pid2_set_limits(pid2, -50.f, 50.f);
    EXPECT_FLOAT_EQ(pid2.min_out, -50.f);
    EXPECT_FLOAT_EQ(pid2.max_out, 50.f);
}

TEST(PidFuzz, Pid2InitInvalidMaxes) {
    PID2_t pid2;
    EXPECT_DEATH(pid2_init(pid2, 1.f, 2.f, 3.f, 4.f, 5.f, 100.f, -10.f), "")
        << "min_out cannot be greater than max_out.";
}

TEST(PidFuzz, Pid2SetLimitsInvalid) {
    PID2_t pid2;
    pid2_init(pid2, 1.f, 2.f, 3.f, 4.f, 5.f, 0.f, 100.f);
    EXPECT_DEATH(pid2_set_limits(pid2, 100.f, -100.f), "")
        << "new_min_out cannot be greater than new_max_out.";
}

TEST(PidCorrectness, Pid2SingleLoopControlMatchesCalculate) {
    PID2_t pid2, pid2_ref;
    pid2_init(pid2, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);
    pid2_init(pid2_ref, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);

    float sp = 10.f;
    float pv = 2.f;
    float dt = 0.1f;

    float out_ref = pid2_calculate(pid2_ref, sp, pv, dt);
    float out = pid2_single_loop_control(pid2, sp, pv, dt);

    EXPECT_FLOAT_EQ(out, out_ref);
    EXPECT_FLOAT_EQ(pid2.total_out, pid2_ref.total_out);
}

TEST(PidCorrectness, PrescaledPid2InitBasic) {
    Prescaled_PID2_t prescaled;
    prescaled_pid2_init(&prescaled, 5, 1.f, 2.f, 3.f, 4.f, 5.f, -100.f, 100.f);
    EXPECT_EQ(prescaled.prescalar, 5u);
    EXPECT_FLOAT_EQ(prescaled.pid.k_p, 1.f);
    EXPECT_FLOAT_EQ(prescaled.pid.k_i, 2.f);
    EXPECT_FLOAT_EQ(prescaled.pid.k_d, 3.f);
    EXPECT_FLOAT_EQ(prescaled.pid.beta, 4.f);
    EXPECT_FLOAT_EQ(prescaled.pid.yeta, 5.f);
    EXPECT_FLOAT_EQ(prescaled.pid.min_out, -100.f);
    EXPECT_FLOAT_EQ(prescaled.pid.max_out, 100.f);
}

TEST(PidCorrectness, PrescaledPid2SingleLoopControlAccumulates) {
    Prescaled_PID2_t prescaled;
    prescaled_pid2_init(&prescaled, 3, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);

    float sp = 10.f;
    float pv = 2.f;
    float dt = 0.1f;

    // Should not update output until prescalar calls
    for (int i = 0; i < 2; ++i) {
        prescaled_pid2_single_loop_control(&prescaled, sp, pv, dt);
        EXPECT_FLOAT_EQ(prescaled.pid.total_out, 0.f);
    }
    // Third call should update output
    prescaled_pid2_single_loop_control(&prescaled, sp, pv, dt);
    EXPECT_NE(prescaled.pid.total_out, 0.f);
}

TEST(PidCorrectness, Pid2DualLoopControlBasic) {
    PID2_t outer, inner;
    pid2_init(outer, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);
    pid2_init(inner, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);

    float sp = 10.f;
    float outer_pv = 2.f;
    float inner_pv = 1.f;
    float outer_dt = 0.1f;
    float inner_dt = 0.1f;

    float out = pid2_dual_loop_control(outer, inner, sp, outer_pv, inner_pv,
                                       outer_dt, inner_dt);

    // Output should be within limits
    EXPECT_GE(out, inner.min_out);
    EXPECT_LE(out, inner.max_out);
}

TEST(PidCorrectness, Pid2TripleLoopControlBasic) {
    PID2_t outer, middle, inner;
    pid2_init(outer, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);
    pid2_init(middle, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);
    pid2_init(inner, 2.f, 1.f, 0.5f, 1.f, 1.f, -100.f, 100.f);

    float sp = 10.f;
    float outer_pv = 2.f;
    float middle_pv = 1.f;
    float inner_pv = 0.5f;
    float outer_dt = 0.1f;
    float middle_dt = 0.1f;
    float inner_dt = 0.1f;

    float out =
        pid2_triple_loop_control(outer, middle, inner, sp, outer_pv, middle_pv,
                                 inner_pv, outer_dt, middle_dt, inner_dt);

    // Output should be within limits
    EXPECT_GE(out, inner.min_out);
    EXPECT_LE(out, inner.max_out);
}
