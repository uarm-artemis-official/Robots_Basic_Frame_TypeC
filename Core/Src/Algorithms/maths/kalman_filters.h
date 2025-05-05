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

#include "uarm_lib.h"
#include "uarm_math.h"

#define sys_dim 2 // the dimension of state vector
#ifdef sys_dim
#endif

/* define matrix operation functions */
#define mat arm_matrix_instance_f32

#define mat_init  arm_mat_init_f32
#define mat_add   arm_mat_add_f32
#define mat_sub   arm_mat_sub_f32
#define mat_mult  arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv   arm_mat_inverse_f32

#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef enum {
	KF_OK = 0,
	KF_ERR = 1
} KFStatus_t;

typedef struct {
	arm_status mat_status;

	/* state space representation - vectors */
	mat x; // state vector
	mat z; // measurement vector
	mat u; // input vector

	/* state space representation - vector dims */
	uint8_t x_size;
	uint8_t z_size;
	uint8_t u_size;

	/* state space representation - matrix */
	mat A; // state transition Matrix
	mat B; // input transition Matrix
    mat H; // measurement matrix

    /* estimation vectors */
    mat xhat;     // state estimate vector
    mat xhat_pri; // state prior estimate vector

    /* noises (the actual noise already included in the input) */
    mat Q;  // process noise covariance matrix
    mat R;  // measurement noise covariance matrix

    /* error */
    mat P;     // est error covariance matrix, where error = (x_k - xhat_k)
    mat P_pri; // prior est error covariance matrix

    /* kalman gain */
    mat K; // kalman gain

    /* intermediate temporary vectors */
    mat temp_mat0; // temp mats
    mat temp_mat1;
    mat temp_mat2;
    mat temp_mat3;
    mat temp_vector0; // temp vectors
    mat temp_vector1;
    mat temp_vector2;

}KalmanFilter_t;

/** ******************************************************************************
  * @file    first order kalman main structure                                                            *
  * @author  Liu heng                                                            *
  * @version V1.0.0                                                              *
  * @date    27-August-2013                                                      *
  * @brief   Hearder file for kalman filter                                      *                                                                        *
  ********************************************************************************/
typedef struct {
    float X_last; // Optimal result of the last moment
    float X_mid;  // Predicted result of the current moment
    float X_now;  // Optimal result of the current moment
    float P_mid;  // Covariance of the predicted result of the current moment
    float P_now;  // Covariance of the optimal result of the current moment
    float P_last; // Covariance of the optimal result of the last moment
    float kg;     // Kalman gain
    float A;      // System parameter
    float Q;      // Process noise covariance
    float R;      // Measurement noise covariance
    float H;      // Measurement matrix
}kalman_filter_t;

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
