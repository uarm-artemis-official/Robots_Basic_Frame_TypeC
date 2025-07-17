/**
 * @file pid.cpp
 * @brief Implementation of all control PID systems.
 * @details All implementations for control systems that utilizes PID as the primary controller
 * are included in this file. This includes single and cascade control loops as well as
 * different implementations of PID with their own solutions to problems known about
 * PID controllers in real systems (e.g. integator windup, derivative kick, etc.).
 * @author James Fu
 * @date 2025-07-17
 */

#include "pid.h"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

/**
 * @brief Initializes a PID_t structure with provided parameters.
 * 
 * @param[out] pid Pointer to PID_t struct to initialize.
 * @param[in] max_out Maximum output magnitude. Must be non-negative.
 * @param[in] max_i_out Maximum integral output magnitude. Must be non-negative.
 * @param[in] max_err Maximum error value. Must be non-negative. Errors greater than this will skip a PID iteration.
 * @note Errors greater than max_err will skip a PID iteration.
 * @note max_out, max_i_out, and max_err must be non-negative.
 * @param[in] kp Proportional gain.
 * @param[in] ki Integral gain.
 * @param[in] kd Derivative gain.
 * @deprecated This PID implementation has known problems and will be removed in the future.
 */
void pid_param_init(PID_t* pid, int32_t max_out, float max_i_out, float max_err,
                    float kp, float ki, float kd) {
    ASSERT(max_out >= 0, "max_out has to be non-negative.");
    ASSERT(max_err >= 0, "max_err has to be non-negative.");
    ASSERT(max_i_out >= 0, "max_i_out has to be non-negative.");

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->cur_val = 0.f;
    pid->target_val = 0.f;

    pid->err = 0.f;
    pid->last_err = 0.f;
    pid->llast_err = 0.f;

    pid->pout = 0.f;
    pid->iout = 0.f;
    pid->dout = 0.f;
    pid->max_out = max_out;
    pid->max_err = max_err;
    pid->max_i_out = max_i_out;
    pid->total_out = 0.f;
}

/**
 * @brief Computes a single PID iteration for one time-step.
 *
 * @param[in, out] pid Pointer to PID_t struct.
 * @param[in] cur_val Plant value.
 * @param[in] target_val Target value.
 * @param[in] dt Time step (seconds).
 * @warning @p dt parameter is ignored and the time-step is always assumed to be 1.0.
 * @return PID calculated output.
 * @note If error exceeds @p max_err, the output is zero and PID iteration calculations are skipped.
 * @deprecated This PID implementation has known problems and will be removed in the future.
 */
float pid_calculate(PID_t* pid, float cur_val, float target_val, float dt) {
    dt = 1.0f;  // Sampling time (fixed)
    pid->cur_val = cur_val;
    pid->target_val = target_val;
    pid->last_err = pid->err;
    pid->err = target_val - cur_val;
    if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
        return 0;
    pid->pout = pid->kp * pid->err;
    pid->iout += pid->ki * pid->err * dt;
    pid->dout = pid->kd * (pid->err - pid->last_err) / dt;

    abs_limit(&(pid->iout), pid->max_i_out);
    pid->total_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->total_out), pid->max_out);

    return pid->total_out;
}

/**
 * @brief Calculates incremental PID output for one time-step.
 *
 * @param[in, out] pid Pointer to PID_t controller struct.
 * @param[in] cur_val Plant value.
 * @param[in] target_val Target value.
 * @return PID calculated output.
 * @note If error exceeds @p max_err, the output is zero and PID iteration is skipped.
 * @note Implements dead zone management: output below 10 is set to zero.
 * @deprecated This PID implementation has known problems and will be removed in the future.
 */
float pid_incremental_calculate(PID_t* pid, float cur_val, float target_val) {
    float dt = 1.0f;  // Sampling time (fixed)
    pid->cur_val = cur_val;
    pid->target_val = target_val;
    pid->llast_err = pid->last_err;
    pid->last_err = pid->err;
    pid->err = target_val - cur_val;
    if ((pid->max_err != 0) && (fabs(pid->err) > pid->max_err))
        return 0;

    pid->pout = pid->kp * (pid->err - pid->last_err);
    pid->iout += pid->ki * dt;
    pid->dout =
        pid->kd * (pid->err - 2.0f * pid->last_err + pid->llast_err) / dt;

    abs_limit(&(pid->iout), pid->max_i_out);
    pid->total_out = pid->pout + pid->iout + pid->dout;
    abs_limit(&(pid->total_out), pid->max_out);

    // PID dead zone risk management
    if (fabs(pid->total_out) < 10)
        pid->total_out = 0;

    return pid->total_out;
}

/**
 * @brief Calculates iteration of single-loop PID controller.
 *
 * @param[in] target_val Target value.
 * @param[in, out] pid Pointer to PID_t controller struct.
 * @param[in] cur_val Plant value.
 * @param[in] dt Time step.
 * @return PID calculated output.
 * @note Calls pid_calculate for computation.
 * @deprecated This PID implementation has known problems and will be removed in the future.
 */
float pid_single_loop_control(float target_val, PID_t* pid, float cur_val,
                              float dt) {
    return pid_calculate(pid, cur_val, target_val, dt);
}

/**
 * @brief Calculate an iteration of dual-loop cascade PID controller.
 *
 * @param[in] f_tar_val Outer loop target value.
 * @param[in, out] f_pid Pointer to outer loop PID_t controller struct.
 * @param[in, out] s_pid Pointer to inner loop PID_t controller struct.
 * @param[in] f_cur_val Outer loop current value.
 * @param[in] s_cur_val Inner loop current value.
 * @param[in] dt Time step for both loops.
 * @return PID calculated output from inner loop.
 * @deprecated This PID implementation has known problems and will be removed in the future.
 */
float pid_dual_loop_control(float f_tar_val, PID_t* f_pid, PID_t* s_pid,
                            float f_cur_val, float s_cur_val, float dt) {
    float f_out = 0;
    f_out = pid_calculate(f_pid, f_cur_val, f_tar_val, dt);
    return pid_calculate(s_pid, s_cur_val, f_out, dt);
}

// TODO: Implement back-calculation.
/**
 * @brief Initializes a PID2_t controller.
 *
 * @param[in, out] pid Reference to PID2_t controller struct to initialize.
 * @param[in] k_p Proportional gain.
 * @param[in] k_i Integral gain.
 * @param[in] k_d Derivative gain.
 * @param[in] beta Proportional error scaling factor.
 * @param[in] yeta Derivative error scaling factor.
 * @note @p beta and @p yeta are parameters for setpoint weighting, which is primarily used for negating effects of derivative kick.
 * @param[in] min_out Minimum output value.
 * @param[in] max_out Maximum output value. Must be greater or equal to min_out.
 */
void pid2_init(PID2_t& pid, float k_p, float k_i, float k_d, float beta,
               float yeta, float min_out, float max_out) {
    ASSERT(max_out >= min_out, "max_out has to be greater or equal to min_out");

    pid.k_p = k_p;
    pid.k_i = k_i;
    pid.k_d = k_d;
    pid.beta = beta;
    pid.yeta = yeta;

    pid.plant_value = 0;
    pid.setpoint = 0;

    pid.p_error = 0;
    pid.i_error = 0;
    pid.d_error = 0;
    pid.prev_d_error = 0;

    pid.p_out = 0;
    pid.i_out = 0;
    pid.d_out = 0;

    pid.max_out = max_out;
    pid.min_out = min_out;

    pid.prev_total_out = 0;
    pid.total_out = 0;
}

/**
 * @brief Sets new output limits for PID2_t controller.
 *
 * @param[in, out] pid Reference to PID2_t controller struct.
 * @param[in] new_min_out New minimum output value.
 * @param[in] new_max_out New maximum output value. Must be greater or equal to new_min_out.
 * @warning Anti-integrator windup is not yet implemented.
 * @todo Implement anti-integrator windup with @p new_min_out and @p new_max_out.
 */
void pid2_set_limits(PID2_t& pid, float new_min_out, float new_max_out) {
    ASSERT(new_max_out >= new_min_out,
           "new_max_out has to be greater or equal to new_min_out.");

    pid.max_out = new_max_out;
    pid.min_out = new_min_out;

    // TODO: Implement anti-integrator windup
}

/**
 * @brief Calculates PID2 output for one time-step.
 *
 * @param[in, out] pid Reference to PID2_t struct.
 * @param[in] sp Setpoint value.
 * @param[in] pv Plant value.
 * @param[in] dt Time step. Must be positive.
 * @return PID2 calculated output.
 * @warning This PID implementation has a rudimentary anti-integrator windup. Back-calculation is not implemented.
 */
float pid2_calculate(PID2_t& pid, float sp, float pv, float dt) {
    ASSERT(dt > 0, "PID must have a timestep.");
    pid.plant_value = pv;
    pid.setpoint = sp;
    pid.prev_d_error = pid.d_error;
    pid.prev_total_out = pid.total_out;

    // Calculate error terms.
    pid.p_error = pid.beta * sp - pv;
    pid.i_error = sp - pv;
    pid.d_error = pid.yeta * sp - pv;

    pid.p_out = pid.k_p * pid.p_error;

    // Anti-integrator windup
    if (pid.min_out < pid.prev_total_out && pid.prev_total_out < pid.max_out) {
        pid.i_out += pid.k_i * pid.i_error * dt;
    }

    pid.d_out = pid.k_d * (pid.d_error - pid.prev_d_error) / dt;
    pid.total_out = pid.p_out + pid.i_out + pid.d_out;

    pid.total_out = value_limit(pid.total_out, pid.min_out, pid.max_out);
    return pid.total_out;
}

/**
 * @brief Caculate an iteration of a single-loop PID2 controller.
 *
 * @param[in, out] pid Reference to PID2_t controller struct.
 * @param[in] sp Setpoint value.
 * @param[in] pv Plant value.
 * @param[in] dt Time step.
 * @return PID2 calculated output.
 * @note This is a wrapper that relies on @ref pid2_calculate to do calculations.
 */
float pid2_single_loop_control(PID2_t& pid, float sp, float pv, float dt) {
    return pid2_calculate(pid, sp, pv, dt);
}

/**
 * @brief Calculates an iteration of a dual-loop PID2 controller.
 *
 * @param[in, out] f_pid Reference to outer loop PID2_t struct.
 * @param[in, out] s_pid Reference to inner loop PID2_t struct.
 * @param[in] sp Setpoint value for outer loop.
 * @param[in] f_pv Plant value for outer loop.
 * @param[in] s_pv Plant value for inner loop.
 * @param[in] f_dt Time step for outer loop.
 * @param[in] s_dt Time step for inner loop.
 * @return PID2 calculated output from inner loop.
 * @note Calculations are done with @ref pid2_calculate.
 */
float pid2_dual_loop_control(PID2_t& f_pid, PID2_t& s_pid, float sp, float f_pv,
                             float s_pv, float f_dt, float s_dt) {
    float f_out = pid2_calculate(f_pid, sp, f_pv, f_dt);
    return pid2_calculate(s_pid, f_out, s_pv, s_dt);
}

/**
 * @brief Calculates an iteration for a triple-loop PID2 controller.
 *
 * @param[in, out] outer_pid Reference to outer loop PID2_t struct.
 * @param[in, out] middle_pid Reference to middle loop PID2_t struct.
 * @param[in, out] inner_pid Reference to inner loop PID2_t struct.
 * @param[in] sp Setpoint value for outer loop.
 * @param[in] outer_pv Plant value for outer loop.
 * @param[in] middle_pv Plant value for middle loop.
 * @param[in] inner_pv Plant value for inner loop.
 * @param[in] outer_dt Time step for outer loop.
 * @param[in] middle_dt Time step for middle loop.
 * @param[in] inner_dt Time step for inner loop.
 * @return PID2 calculated output from inner loop.
 * @note Uses @ref pid2_calculate for each loop.
 */
float pid2_triple_loop_control(PID2_t& outer_pid, PID2_t& middle_pid,
                               PID2_t& inner_pid, float sp, float outer_pv,
                               float middle_pv, float inner_pv, float outer_dt,
                               float middle_dt, float inner_dt) {
    float outer_out = pid2_calculate(outer_pid, sp, outer_pv, outer_dt);
    float middle_out =
        pid2_calculate(middle_pid, outer_out, middle_pv, middle_dt);
    return pid2_calculate(inner_pid, middle_out, inner_pv, inner_dt);
}

/**
 * @brief Initializes a Prescaled_PID2_t structure with provided parameters.
 *
 * @param prescaled Pointer to Prescaled_PID2_t struct to initialize.
 * @param prescalar Number of calls before PID2 calculation is performed.
 * @param k_p Proportional gain.
 * @param k_i Integral gain.
 * @param k_d Derivative gain.
 * @param beta Proportional error scaling factor.
 * @param yeta Derivative error scaling factor.
 * @note @p beta and @p yeta are parameters for setpoint weighting, which is primarily used for negating effects of derivative kick.
 * @param min_out Minimum output value.
 * @param max_out Maximum output value.
 */
void prescaled_pid2_init(Prescaled_PID2_t* prescaled, uint32_t prescalar,
                         float k_p, float k_i, float k_d, float beta,
                         float yeta, float min_out, float max_out) {
    prescaled->prescalar = prescalar;
    prescaled->prescalar_count = 0;
    prescaled->cumsum_dt = 0;

    pid2_init(prescaled->pid, k_p, k_i, k_d, beta, yeta, min_out, max_out);
}

/**
 * @brief Calculate an iteration for prescaled single-loop PID2 controller.
 *
 * @param prescaled Pointer to Prescaled_PID2_t struct.
 * @param sp Setpoint value.
 * @param pv Plant (process) value.
 * @param dt Time step.
 * @note PID2 calculation is performed only after prescalar calls.
 * @note Uses @ref pid2_calculate for calculations. 
 * @note @ref pid2_calculate is called with an accumulated time step cumsum_dt.
 */
void prescaled_pid2_single_loop_control(Prescaled_PID2_t* prescaled, float sp,
                                        float pv, float dt) {
    prescaled->prescalar_count++;
    prescaled->cumsum_dt += dt;
    if (prescaled->prescalar_count >= prescaled->prescalar) {
        pid2_calculate(prescaled->pid, sp, pv, prescaled->cumsum_dt);
        prescaled->prescalar_count = 0;
        prescaled->cumsum_dt = 0;
    }
}
