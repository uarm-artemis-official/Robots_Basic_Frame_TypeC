/*******************************************************************************
* @file           : maths.c
* @brief(op)      : general use of the maths include:
* 	                - math computing functions
* 	                - generic filters like low pass, high pass, comp pass, etc.
* @created time	  : Jul, 2023
* @author(op)     : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __MATHS_C__
#define __MATHS_C__

#include "uarm_math.h"
#include "uarm_lib.h"
#include "uarm_defines.h"
#include "string.h"
#include "float.h"


/*********** For genral use of the maths **********/
/**
  * @brief     exponential sum of the array input
  * @param[in] input array with raw data
  * @param[in] the seq number of the array
  * @retval    exp sum
  */
double exp_sum(double *array, int length) {
    double sum = 0.0;
    for (int i = 0; i < length; ++i) {
        sum += exp(array[i]);
    }
    return sum;
}

/**
  * @brief     softmax implementation
  * @param[in] input array with raw data
  * @param[in] the seq number of the array
  * @retval    None
  */
void softmax(float* x, int len) {
    // Compute the maximum value of the input vector to prevent overflow in exp.
    float max_value = -FLT_MAX;
    for (int i = 0; i < len; i++) {
        if (x[i] > max_value) {
            max_value = x[i];
        }
    }

    // Subtract the maximum value from all elements for numerical stability, then exponentiate.
    float exp_sum = 0.0;
    for (int i = 0; i < len; i++) {
        x[i] = exp(x[i] - max_value);
        exp_sum += x[i];
    }

    // Normalize the vector to make it a valid probability distribution.
    for (int i = 0; i < len; i++) {
        x[i] /= exp_sum;
    }
}

/**
  * @brief  absolute limitation
  * @param a Number to be limited to.
  * @param abs_limit Limit of the magnitude that `a` can be. Has to be non-negative.
  * @retval None
  * 
  * This function limits the magnitude of a floating point number to within a limit. 
  * Values above (for positive numbers) or below (for negative numbers) abs_limit 
  * will be truncated down to abs_limit. Values within [-abs_limit, abs_limit] will
  * remain unchanged.
  */
void abs_limit(float *a, float abs_limit)
{
    ASSERT(abs_limit >= 0, "abs_limit must be non-negative");
    if (*a > abs_limit) *a = abs_limit;
    if (*a < -abs_limit) *a = -abs_limit;
}

/**
  * @brief  Maps an input value from one domain to another.
  * @retval mapped radians
  * 
  * This linearly transforms a value from one domain to another.
  * e.g. degrees ([0, 360]) to radians ([0, 2pi]).
  */
float in_out_map(float input, float in_min, float in_max, float out_min, float out_max){
    ASSERT(out_max >= out_min, "out_max should be greater than out_min");
    ASSERT(in_max >= in_max, "in_max should be greater than in_min.");
    ASSERT(input >= in_min && input <= in_max, "input has to be in [in_min, in_max].");
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/******************************************************************************
* @func groups    : filters
* @brief          : Some filters for data process and normailzation
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************/
void init_folp_filter(first_order_low_pass_t *folp, float a){
	folp->a = a;
	folp->cur_data = 0;
	folp->last_output_data = 0;
	folp->output_data = 0;
}
/**
  * @brief     First order low pass filter
  * @param[in] main struct
  * @param[in] raw data needed to be filtered
  * @Note
  * 			Equation: Y(n)=a*X(n)+(1-a)*Y(n-1), weighted sum
  * 			hyperparameter: a[0,1] -> smaller : stability up, sensitivity down (used when data is slightly oscillating)
  * 								 	  larger  : stability down, sensitivity up (used when data rapidly changes)
  * @attention significant phase lags. Good suppression of periodic disturbances tho.
  * @retval    output
  */
float first_order_low_pass_filter(first_order_low_pass_t *folp, float data){
	folp->cur_data = data;
	/* apply equation */
	folp->output_data = folp->a * folp->cur_data + (1.0-folp->a)*folp->last_output_data;
	folp->last_output_data = folp->output_data;
	return folp->output_data;
}

/**
  * @brief     First order high pass filter
  * @param[in] main struct
  * @param[in] raw data needed to be filtered
  * @Note
  * 			Equation: Y(n)=a*Y(n-1)+ a*(X(n)-X(n-1))
  * 			hyperparameter: a[0,1] -> smaller : stability up, senstivity down (used when data is slightly oscillating)
  * 								 	  larger  : stability down, senstivity up (used when data rapidly changes)
  * @attention Not suitable for slowly changed data.
  * @retval    output
  */
float first_order_high_pass_filter(first_order_high_pass_t *fohp, float data){
	fohp->input_data = data;
	/* apply equation */
	fohp->output_data = fohp->a * fohp->last_output_data + (fohp->a)*(fohp->input_data - fohp->last_input_data);
	fohp->last_input_data = fohp->input_data;
	fohp->last_output_data = fohp->output_data;

	return fohp->output_data;
}

/**
  * @brief     First order Complementary filter
  * @param[in] hyperparameter: a[0,1]
  * @param[in] low pass value
  * @param[in] high pass value
  * @Note
  * 			Equation: Y(n)=a*low + (1-a)*high
  * 			hyperparameter: a[0,1]
  * @retval    output
  */
float first_order_comp_filter(float a,  float lowPF, float highPF){
	return (a*lowPF + (1.0-a)*highPF);
}

void init_ewma_filter(ewma_filter_t *ewma, float a){
	ewma->a = a;
	ewma->output_data = 0;
	ewma->last_output_data = 0;
}
/* estimate data */
/**
  * @brief     Exponentially Weighted Moving Average filter(EWMA)
  * @param[in] main ewna struct
  * @param[in] current value to be filtered
  * @Note	   output = alpha * reading + (1 - alpha) * lastOutput, seems better than mean sliding filter
  * @retval    output
  */
float ewma_filter(ewma_filter_t *ewma, float cur_data) {
	ewma->output_data = ewma->a * cur_data + (1.0 - ewma->a)*ewma->last_output_data;
	ewma->last_output_data = ewma->output_data;
    return ewma->output_data;
}


void init_swm_filter(sliding_mean_filter_t *filter, size_t window_size){
    /* window size should not be greater than max window size */
	filter->window_size = (window_size <= MAX_WINDOW_SIZE) ? window_size : MAX_WINDOW_SIZE;
    filter->current_index = 0;
    filter->sum = 0.0f;
    memset(filter->window, 0, filter->window_size * sizeof(float));
}
/**
 * @brief     Sliding Window Mean Filter
 * @param[in] struct containing filter parameters
 * @param[in] new data to be filtered
 * @Note
 *      Function: Keeps a window of the last N data,
 *                calculates the average of the data in this window at each step.
 *      Use case: Useful for reducing noise from fluctuating data.
 *
 * @attention Make sure that the window size parameter is set according to the requirement.
 * @retval    filtered output (mean of the window)
 */
float sliding_window_mean_filter(sliding_mean_filter_t *filter, float new_data){
    /* subtract the oldest data from sum */
    filter->sum -= filter->window[filter->current_index];
    /* add the new data to the sum */
    filter->sum += new_data;
    /* replace the oldest data with the new data */
    filter->window[filter->current_index] = new_data;
    /* increment the index, wrap around if necessary */
    filter->current_index = (filter->current_index + 1) % filter->window_size;

    return filter->sum / filter->window_size;
}




#endif
