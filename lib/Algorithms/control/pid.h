/**
 * There are different forms of PID controllers depending on how the 
 * governing equations of the controller are defined. Here are some:
 * 
 * (Parallel form) 
 * u(t) = K_p * e(t) + K_i * int^t_0 e(\tau) d\tau + K_d * de(t)/dt
 * 
 * There are individual gains per PID segment (i.e. there is one 
 * proportional gain, one integral gain, and one derivative gain).
 * This allows for tuning of each individual segment independently
 * from one another. Critism of this form is that gains are not tied
 * to any physical meaning and are just numbers that have to be toggled.
 * 
 * (Standard form)
 * u(t) = K_p * [e(t) + 1/T_i \int^t_0 e(\tau) d\tau + T_d * de(t)/dt]
 * 
 * Commonly used form in industry where K_p term is factored out and
 * applied to all segments of the controller. Gains in this form are 
 * considered more grounded in physical reality with K_p considered a 
 * scaling term, T_i is integral time (how long to eliminate past errors),
 * T_d is derivative time (how far into the future should be considered to 
 * predict error value). Cohen-Coon and Lambda tuning methods were developed
 * for this form.
 * 
 * (Series form)
 * An older form used in pneumatic and electronic controllers. 
 * Ziegler-Nichols tuning method was developed for this form.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "control_types.hpp"

// TODO Convert to structs or classes.
// TODO: Implement back-calculation + pseudo-derivative.

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
               float yeta, float min_out, float max_out);

void pid2_init(PID2_t& pid, pid::PID2Config config);

/**
 * @brief Sets new output limits for PID2_t controller.
 *
 * @param[in, out] pid Reference to PID2_t controller struct.
 * @param[in] new_min_out New minimum output value.
 * @param[in] new_max_out New maximum output value. Must be greater or equal to new_min_out.
 * @warning Anti-integrator windup is not yet implemented.
 * @todo Implement anti-integrator windup with @p new_min_out and @p new_max_out.
 */
void pid2_set_limits(PID2_t& pid, float new_min_out, float new_max_out);

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
float pid2_calculate(PID2_t& pid, float sp, float pv, float dt);

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
float pid2_single_loop_control(PID2_t& pid, float sp, float pv, float dt);

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
                             float s_pv, float f_dt, float s_dt);

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
                               float middle_dt, float inner_dt);

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
                         float yeta, float min_out, float max_out);

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
                                        float pv, float dt);

#endif