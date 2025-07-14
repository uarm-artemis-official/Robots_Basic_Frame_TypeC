#ifndef __UARM_TYPES_H
#define __UARM_TYPES_H

#include "stddef.h"
#include "stdint.h"

#include "uarm_defines.h"

/* =========================================================================
 * UARM MATH TYPES
 * ====================================================================== */
typedef struct {
    float cur_data;
    float output_data;
    float last_output_data;

    float a;  //filter parameter
} first_order_low_pass_t;

typedef struct {
    float input_data;
    float output_data;
    float last_input_data;
    float last_output_data;

    float a;  //filter parameter
} first_order_high_pass_t;

typedef struct {
    float output_data;
    float last_output_data;

    float a;  //filter parameter
} ewma_filter_t;

typedef struct {
    float window[MAX_WINDOW_SIZE];  // buffer to the data window
    size_t window_size;             // size of the window
    size_t current_index;           // current position in the window
    float sum;                      // sum of the window's elements
} sliding_mean_filter_t;

/* =========================================================================
 * UARM SI UNIT STRONG TYPES
 * ====================================================================== */

#endif