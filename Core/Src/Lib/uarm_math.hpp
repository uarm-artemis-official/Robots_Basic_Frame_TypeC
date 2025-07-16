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

#ifndef __UARM_MATH_HPP
#define __UARM_MATH_HPP

#ifdef GTEST
#include "fake_arm_math.h"
#else
#include "arm_math.h"
#include "stm32f407xx.h"
#endif

// #include <cmath>
#include "math.h"
#include "uarm_types.hpp"

/* User defiend functions declaration */
/* maths */
float sign(float x);

double exp_sum(double* array, int length);
void softmax(float* x, int len);
void abs_limit(float* a, float ABS_MAX);
float in_out_map(float input, float in_min, float in_max, float out_min,
                 float out_max);
float fast_inv_sqrt(float x);
float realign(float theta, float pos);

float value_limit(float x, float min_x, float max_x);
float relative_angle(float angle1, float angle2);
float relative_angle_rad(float angle1, float angle2);

/* generic filters */
void init_folp_filter(first_order_low_pass_t* folp, float a);
float first_order_low_pass_filter(first_order_low_pass_t* folp, float data);
float first_order_high_pass_filter(first_order_high_pass_t* fohp, float data);
float first_order_comp_filter(float a, float lowPF, float highPF);
void init_ewma_filter(ewma_filter_t* ewma, float a);
float ewma_filter(ewma_filter_t* ewma, float cur_data);
void init_swm_filter(sliding_mean_filter_t* filter, size_t window_size);
float sliding_window_mean_filter(sliding_mean_filter_t* filter, float new_data);

#endif
