/*******************************************************************************
* @file           : kalman_filers.h
* @brief          : integrate first/second order kalman filters and EKF.
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __KALMAN_FILTERS_H__
#define __KALMAN_FILTERS_H__


#include "stm32f407xx.h"
#include "stdlib.h"
#include "FreeRTOS.h"
#include "maths_types.h"

/* functions declaration here */
void kf_mat_restruct(mat *mat, uint8_t rows, uint8_t cols);
void kf_mat_reset(mat *mat, uint8_t rows, uint8_t cols);
void kf_param_init(KalmanFilter_t *kf, uint8_t xSize, uint8_t uSize, uint8_t zSize );
void kf_update_input(KalmanFilter_t *kf, mat *input);
void kf_set_mat(mat *mat_target, mat *mat_source);
KFStatus_t kf_data_fusion(KalmanFilter_t *kf);
KFStatus_t kf_execute(KalmanFilter_t *kf, mat *input);
void kf_param_deinit(KalmanFilter_t *kf);

/*scalar kalman func declares */
void kalmanCreate(kalman_filter_t *p,float T_Q,float T_R);
float KalmanFilter(kalman_filter_t* p,float dat);

#endif /*__KALMAN_FILTERS_H__*/
