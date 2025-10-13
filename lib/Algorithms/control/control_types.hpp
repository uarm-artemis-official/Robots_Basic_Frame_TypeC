#ifndef __CONTROL_TYPES_HPP
#define __CONTROL_TYPES_HPP

#include "uarm_types.hpp"

/* =========================================================================
 * FEED FORWARD TYPES
 * ====================================================================== */
/**
 * @brief Feed-forward control structure
 *
 * Contains parameters and state variables for implementing feed-forward control.
 */
typedef struct {
    float ff_gain;    /**< Feed-forward gain coefficient */
    float last_input; /**< Previous input value */
    float output;     /**< Current feed-forward output value */
} FeedForward_t;

/* =========================================================================
 * PID TYPES
 * ====================================================================== */

/**
 * @brief Enhanced PID controller structure with additional parameters
 *
 * Implements a PID controller with beta and yeta parameters for setpoint weighting.
 * Yeta may be used to reduce the effects of derivative kick.
 * See this article for more details (note: yeta is mislabelling of the gamma coefficient): 
 * https://jckantor.github.io/CBE30338/04.02-PID_Control_with_Setpoint_Weighting.html
 */
typedef struct {
    float k_p;  /**< Proportional gain coefficient */
    float k_i;  /**< Integral gain coefficient */
    float k_d;  /**< Derivative gain coefficient */
    float beta; /**< Setpoint weighting coefficient */
    float yeta; /**< Setpoint weighting coefficient */

    float plant_value; /**< Current process value */
    float setpoint;    /**< Desired setpoint value */

    float p_error;      /**< Proportional error term */
    float i_error;      /**< Integral error term */
    float d_error;      /**< Derivative error term */
    float prev_d_error; /**< Previous derivative error */

    float p_out; /**< Proportional term output */
    float i_out; /**< Integral term output */
    float d_out; /**< Derivative term output */

    float max_out; /**< Maximum output limit */
    float min_out; /**< Minimum output limit */

    float prev_total_out; /**< Previous total output */
    float total_out;      /**< Current total output */
} PID2_t;

/**
 * @brief Another PID implementation with intent to replace @ref PID2_t.
 * 
 * @todo Add pseudo-derivative filter to reduce noise sensitivity.
 */
typedef struct {
    float k_p;   /**< Proportional gain coefficient */
    float k_i;   /**< Integral gain coefficient */
    float k_d;   /**< Derivative gain coefficient */
    float beta;  /**< Setpoint weighting coefficient */
    float gamma; /**<Setpoint weighting coefficient */

    float plant_value; /**< Current process value */
    float setpoint;    /**< Desired setpoint value */

    float p_error;      /**< Proportional error term */
    float i_error;      /**< Integral error term */
    float d_error;      /**< Derivative error term */
    float prev_d_error; /**< Previous derivative error */

    float p_out; /**< Proportional term output */
    float i_out; /**< Integral term output */
    float d_out; /**< Derivative term output */

    float max_out; /**< Maximum output limit */
    float min_out; /**< Minimum output limit */

    float prev_total_out; /**< Previous total output */
    float total_out;      /**< Current total output */
} PID3_t;

/**
 * @brief Prescaled PID controller for different frequency control loops
 *
 * Allows for control loops of different frequencies. For example, while the IMU_App
 * runs at 1000 Hz, its temperature control might run at 100 Hz.
 * 
 * @note Only @ref PID2_t can be used for prescaled PID control.
 * @see imu_temp_pid_control() in IMU_App.cpp.
 */
typedef struct {
    PID2_t pid;               /**< The base PID controller structure */
    float cumsum_dt;          /**< Cumulative sum of time steps */
    uint32_t prescalar;       /**< Frequency divider value */
    uint32_t prescalar_count; /**< Current prescalar counter value */
} Prescaled_PID2_t;

/* =========================================================================
 * RAMP TYPES
 * ====================================================================== */
/**
 * @brief Ramp generator for smooth transitions
 *
 * Generates a ramp signal to limit rate of change between values.
 * These are used in motion control of chassis wheels to prevent slip during
 * sudden acceleration or deceleration while in pc mode.
 */
typedef struct {
    float target;     /**< Target value to ramp to */
    float start;      /**< Starting value of the ramp */
    float max_change; /**< Maximum allowed change per time step */
    float output;     /**< Current output value */
    float cumsum_dt;  /**< Cumulative sum of time steps */
} Ramp;

#endif