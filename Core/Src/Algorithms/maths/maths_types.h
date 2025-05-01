#ifndef __MATHS_TYPES_H
#define __MATHS_TYPES_H

#include "maths_defines.h"

/* =========================================================================
 * KALMAN FILTERS TYPES
 * ====================================================================== */

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

/* =========================================================================
 * MATHS TYPES
 * ====================================================================== */
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

#endif