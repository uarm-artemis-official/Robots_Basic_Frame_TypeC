/*
******************************************************************************
* @file           : maths.h
* @brief(op)      : general use of the maths
* @created time	  : Jul, 2023
* @author(op)     : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/

#ifndef __MATHS_H__
#define __MATHS_H__

#include "math.h"
#include "arm_math.h"
#include "float.h"

#define DEGREE2RAD 0.0174533f
#define RAD2DEGEE  57.3f

#define MAX_WINDOW_SIZE 300

// Taken from: https://stackoverflow.com/questions/3437404/min-and-max-in-c
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define VAL_LIMIT(val, min, max) \
  do                             \
  {                              \
    if ((val) <= (min))          \
    {                            \
      (val) = (min);             \
    }                            \
    else if ((val) >= (max))     \
    {                            \
      (val) = (max);             \
    }                            \
  } while (0)

#define ANGLE_LIMIT_360(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
  do                                \
  {                                 \
    if((val)>180)                   \
      (val) -= 360;                 \
  } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))


/* define maths' struct */
typedef struct{
	float cur_data;
	float output_data;
	float last_output_data;

	float a; //filter parameter
}first_order_low_pass_t;

typedef struct{
	float input_data;
	float output_data;
	float last_input_data;
	float last_output_data;

	float a; //filter parameter
}first_order_high_pass_t;

typedef struct{
	float output_data;
	float last_output_data;

	float a; //filter parameter
}ewma_filter_t;

typedef struct
{
    float  window[MAX_WINDOW_SIZE];  // buffer to the data window
    size_t window_size;  			 // size of the window
    size_t current_index;  			 // current position in the window
    float  sum;           			 // sum of the window's elements
}sliding_mean_filter_t;


/* User defiend functions declaration */
/* maths */
double exp_sum(double *array, int length);
void softmax(float* x, int len);
void abs_limit(float *a, float ABS_MAX);
float in_out_map(float input, float in_min, float in_max, float out_min, float out_max);

/* generic filters */
void init_folp_filter(first_order_low_pass_t *folp, float a);
float first_order_low_pass_filter(first_order_low_pass_t *folp, float data);
float first_order_high_pass_filter(first_order_high_pass_t *fohp, float data);
float first_order_comp_filter(float a,  float lowPF, float highPF);
void init_ewma_filter(ewma_filter_t *ewma, float a);
float ewma_filter(ewma_filter_t *ewma, float cur_data);
void init_swm_filter(sliding_mean_filter_t *filter, size_t window_size);
float sliding_window_mean_filter(sliding_mean_filter_t *filter, float new_data);


#endif
