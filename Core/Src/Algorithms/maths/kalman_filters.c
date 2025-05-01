/*******************************************************************************
* @file           : kalman_filers.c
* @brief          : integrate first/second order kalman filters and EKF.
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __KALMAN_FILTERS_C__
#define __KALMAN_FILTERS_C__

#include "kalman_filters.h"
#include "maths_defines.h"

/****************************************************************************************************
* Kalman filter explanation and examples for gyroscope (assumed to be linear) by Haoran
*****************************************************************************************************
* Assume we have a linear system wit hstate space representation:
* 	X_k = A_k * X_{k-1} + B_k * u_k + W_k, where p(W_k)~N(0,Q), Q is the Process Noise Covariance Matrix
* 	Z_k = H_k * X_k + V_k, 					 where p(V_k)~N(0,R), R is the Measurement Noise Covariance Matrix
*
* 	Kalman filter 5 core equations:
* 	 <------------------------------------- estimation ------------------------------------->
* 	 xhat_pri = A*xhat_last + B*u_last  -> No.1 formula
* 	 P_pri = A*P_last*A^T + Q 			-> No.2 formula, P is the est error covariance matrix,
* 	 										   where error = (x_k - xhat_k)
* 	 <------------------------------------- calibration ------------------------------------>
* 	 K = H*P_pri*(H*P_pri*H^T + R)^-1 	-> No.3 formula, K is the Kalman Gain
* 	 xhat = xhat_pri + K*(z-H*xhat_pri) -> No.4 formula, xhat is the estimated state, also the result
* 	 P = P_pri - K*H*P_pri 				-> No.5 formula
*
* Usage of Kalman filter code:
* 	1. Initialize Kalman filter:
* 		kf_param_init(KalmanFilter_t *kf, uint8_t xSize, uint8_t uSize, uint8_t zSize )
* 	2. Set Kalman filter MAT -> Q, R, A, B, H, where Q and R determine the est results more close to prior est or measurement:
* 		kf_set_mat(mat *mat_target, mat *mat_source)
* 	3. Excute Kalman filter:
* 		kf_execute(KalmanFilter_t *kf, mat *input)
*
* 	@Attention: This procedure only be used when the size of x is greater than 1.(aka vector instead of scalar),
* 		        otherwise we should use the fisrt order kf open-sourced by Liu.
*
******************************************************************************************************/

/* define internal vars */

/* define internal functions */

/* for gyroscope filters */
/**
  * @brief     restruct the value of the mat back to 0
  * @param[in] main structure of the kalman filter
  * @note      This only wokrs when the size of the mat changed but not change value.
  * @retval    None
  */
void kf_mat_restruct(mat *mat, uint8_t rows, uint8_t cols){
	mat->numRows = rows;
	mat->numCols = cols;
	//FIXME: test if we need to reset to 0
//	memset(mat->pData, 0, rows * cols * sizeof(float));
}
/**
  * @brief     reset the value of the mat back to 0
  * @param[in] main structure of the kalman filter
  * @note      This only wokrs when the size of the mat not changed.
  * @retval    None
  */
void kf_mat_reset(mat *mat, uint8_t rows, uint8_t cols){
	memset(mat->pData, 0, rows * cols * sizeof(float));
}

/**
  * @brief     init function of the kalman filter
  * @param[in] main structure of the kalman filter
  * @param[in]
  * @retval    None
  */
void kf_param_init(KalmanFilter_t *kf, uint8_t xSize, uint8_t uSize, uint8_t zSize ){
	/* specify the vec/mat size */
	kf->x_size = xSize;
	kf->u_size = uSize;
	kf->z_size = zSize;

	/* define max rows and cols for temp mat/vec */
	uint8_t max_size = MAX(MAX(kf->x_size, kf->z_size), kf->u_size);

	/* old version of init, no assign with dynamic mem management */
	/*float32_t temp_x_data1[xSize];
	float32_t temp_x_data2[xSize];
	float32_t temp_u_data[uSize];
	float32_t temp_z_data[zSize];

	mat_init(&kf->xhat, kf->x_size, 1, temp_x_data1); //estimated  state vector x_size*1
	kf_mat_reset(&kf->xhat, kf->x_size, 1);
	mat_init(&kf->xhat_pri, kf->x_size, 1, temp_x_data2); //prior estimated state vector x_size*1
	kf_mat_reset(&kf->xhat_pri, kf->x_size, 1);
	mat_init(&kf->u, kf->u_size, 1, temp_u_data); // input vector u_size*1
	kf_mat_reset(&kf->u, kf->u_size, 1);
	mat_init(&kf->z, kf->z_size, temp_z_data); // state vector z_size*1
	kf_mat_reset(&kf->z, kf->z_size, 1);
	*/

	/* initialize vectors */
	kf->x.pData = (float32_t *)pvPortMalloc(xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->x, xSize, 1, kf->x.pData);
	kf_mat_reset(&kf->x, xSize, 1);

	kf->xhat.pData = (float32_t *)pvPortMalloc(xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->xhat, xSize, 1, kf->xhat.pData);
	kf_mat_reset(&kf->xhat, xSize, 1);

	kf->xhat_pri.pData = (float32_t *)pvPortMalloc(xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->xhat_pri, xSize, 1, kf->xhat_pri.pData);
	kf_mat_reset(&kf->xhat_pri, xSize, 1);

	kf->u.pData = (float32_t *)pvPortMalloc(uSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->u, uSize, 1, kf->u.pData);
	kf_mat_reset(&kf->u, uSize, 1);

	kf->z.pData = (float32_t *)pvPortMalloc(zSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->z, zSize, 1, kf->z.pData);
	kf_mat_reset(&kf->z, zSize, 1);

	/* state space representation - matrix */
	kf->A.pData = (float32_t *)pvPortMalloc(xSize * xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->A, xSize, xSize, kf->A.pData);
	kf_mat_reset(&kf->A, xSize, xSize);

	kf->B.pData = (float32_t *)pvPortMalloc(xSize * uSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->B, xSize, uSize, kf->B.pData);
	kf_mat_reset(&kf->B, xSize, uSize);

	kf->H.pData = (float32_t *)pvPortMalloc(zSize * xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->H, zSize, xSize, kf->H.pData);
	kf_mat_reset(&kf->H, zSize, xSize);

	/* noises (the actual noise already included in the input) */
	kf->Q.pData = (float32_t *)pvPortMalloc(xSize * xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->Q, xSize, xSize, kf->Q.pData);
	kf_mat_reset(&kf->Q, xSize, xSize);

	kf->R.pData = (float32_t *)pvPortMalloc(zSize * zSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->R, zSize, zSize, kf->R.pData);
	kf_mat_reset(&kf->R, zSize, zSize);

	/* error */
	kf->P.pData = (float32_t *)pvPortMalloc(xSize * xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->P, xSize, xSize, kf->P.pData);
	kf_mat_reset(&kf->P, xSize, xSize);

	kf->P_pri.pData = (float32_t *)pvPortMalloc(xSize * xSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->P_pri, xSize, xSize, kf->P_pri.pData);
	kf_mat_reset(&kf->P_pri, xSize, xSize);

	/* kalman gain */
	kf->K.pData = (float32_t *)pvPortMalloc(xSize * zSize * sizeof(float32_t));
	arm_mat_init_f32(&kf->K, xSize, zSize, kf->K.pData);
	kf_mat_reset(&kf->K, xSize, zSize);

	/* intermediate temporary mat/vectors, make sure to apply max value  */
	kf->temp_mat0.pData = (float32_t *)pvPortMalloc(max_size * max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_mat0, max_size, max_size, kf->temp_mat0.pData);
	kf_mat_reset(&kf->temp_mat0, max_size, max_size);

	kf->temp_mat1.pData = (float32_t *)pvPortMalloc(max_size * max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_mat1, max_size, max_size, kf->temp_mat1.pData);
	kf_mat_reset(&kf->temp_mat1, max_size, max_size);

	kf->temp_mat2.pData = (float32_t *)pvPortMalloc(max_size * max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_mat2, max_size, max_size, kf->temp_mat2.pData);
	kf_mat_reset(&kf->temp_mat2, max_size, max_size);

	kf->temp_mat3.pData = (float32_t *)pvPortMalloc(max_size * max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_mat3, max_size, max_size, kf->temp_mat3.pData);
	kf_mat_reset(&kf->temp_mat3, max_size, max_size);

	kf->temp_vector0.pData = (float32_t *)pvPortMalloc(max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_vector0, max_size, 1, kf->temp_vector0.pData);
	kf_mat_reset(&kf->temp_vector0, max_size, 1);

	kf->temp_vector1.pData = (float32_t *)pvPortMalloc(max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_vector1, max_size, 1, kf->temp_vector1.pData);
	kf_mat_reset(&kf->temp_vector1, max_size, 1);

	kf->temp_vector2.pData = (float32_t *)pvPortMalloc(max_size * sizeof(float32_t));
	arm_mat_init_f32(&kf->temp_vector2, max_size, 1, kf->temp_vector2.pData);
	kf_mat_reset(&kf->temp_vector2, max_size, 1);

	/* restruct the size for first data fusion */
	kf_mat_restruct(&kf->temp_mat0, kf->x_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat1, kf->x_size, kf->x_size);
}

/**
  * @brief     update input vector kf->u
  * @param[in] main structure of the kalman filter
  * @param[in] new input vector
  * @retval    None
  */
void kf_update_input(KalmanFilter_t *kf, mat *input){
	/* if dimensions are different, reallocate memory */
	if(kf->u.numRows != input->numRows || kf->u.numCols != input->numCols){
	        vPortFree(kf->u.pData);  // Free old memory
	        kf->u.pData = (float32_t *)pvPortMalloc(sizeof(float32_t) * input->numRows * input->numCols);  // Allocate new memory
	        kf->u.numRows = input->numRows;  // Update dimensions
	        kf->u.numCols = input->numCols;
	    }

	    /* memcpy the values from input to kf->u */
	    memcpy(kf->u.pData, input->pData, input->numRows * input->numCols * sizeof(float32_t));
}

/**
  * @brief     set any matrix
  * @param[in] target mat to be set
  * @param[in] source mat that has set values
  * @retval    None
*/
void kf_set_mat(mat *mat_target, mat *mat_source){
	assert(mat_target->numRows == mat_source->numRows && mat_target->numCols == mat_source->numCols);

	/* set value for target mat */
	memcpy(mat_target->pData, mat_source->pData, mat_source->numRows * mat_source->numCols * sizeof(float));
//	for(int i=0; i < mat_target->numRows; i++) {
//	    for(int j=0; j < mat_target->numCols; j++) {
//	        mat_target->pData[i * mat_target->numCols + j] = mat_source->pData[i * mat_source->numCols + j];
//	    }
//	}
}
/**
  * @brief     init function of the kalman filter
  * @param[in] main structure of the kalman filter
  * @retval    None
  */
KFStatus_t kf_data_fusion(KalmanFilter_t *kf){
	/*** estimate process begins here ***/
	/* update prior estimation, pure vector computes */
	kf_mat_restruct(&kf->temp_vector0, kf->x_size, 1);
	kf_mat_restruct(&kf->temp_vector1, kf->u_size, 1);
	mat_mult(&kf->A, &kf->xhat, &kf->temp_vector0);   // temp A*xhat_last, -> temp_vector0: x_size*1
	mat_mult(&kf->B, &kf->u, &kf->temp_vector1); 	  // temp B*u_last, -> temp_vector1: u_size*1
	kf->mat_status = mat_add(&kf->temp_vector0, &kf->temp_vector1, &kf->xhat_pri); // xhat_pri = A*xhat_last + B*u_last -> No.1 formula, -> xhat_pri: x_size*1

	/* update prior est error covariance matrix -> predict updates */
	kf_mat_restruct(&kf->temp_mat0, kf->x_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat1, kf->x_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat2, kf->x_size, kf->x_size);
	mat_mult(&kf->A, &kf->P, &kf->temp_mat0); 		// temp A*P_last, -> temp_mat0: x_size*x_size
	mat_trans(&kf->A, &kf->temp_mat1);				// temp A^T -> temp_mat1: x_size*x_size
	mat_mult(&kf->temp_mat0, &kf->temp_mat1, &kf->temp_mat2);   // temp A*P_last*A^T -> temp_mat2: x_size*x_size
	kf->mat_status = mat_add(&kf->temp_mat2, &kf->Q, &kf->P_pri); // P_pri = A*P_last*A^T + Q -> No.2 formula -> P_pri: x_size*x_size

	/*** calibration process ***/
	/* update kalman gain */
	kf_mat_restruct(&kf->temp_mat0, kf->z_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat1, kf->x_size, kf->z_size);
	kf_mat_restruct(&kf->temp_mat2, kf->z_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat3, kf->z_size, kf->z_size);
	mat_trans(&kf->H, &kf->temp_mat0);  // temp H^T -> temp_mat0: z_size*x_size
	mat_mult(&kf->P_pri, &kf->temp_mat0, &kf->temp_mat1); //temp P_pri*H^T -> temp_mat1: x_size*z_size
	mat_mult(&kf->H, &kf->P_pri, &kf->temp_mat2); //temp H*P_pri -> temp_mat2: z_size*x_size
	mat_mult(&kf->temp_mat2, &kf->temp_mat0, &kf->temp_mat3); //temp H*P_pri*H^T -> temp_mat3: z_size*z_size
	//reset temp mat 0 to z_size*z_size
	kf_mat_restruct(&kf->temp_mat0, kf->z_size, kf->z_size);
	mat_add(&kf->temp_mat3, &kf->R, &kf->temp_mat0); // temp H*P_pri*H^T + R -> temp_mat0: z_size*z_size
	//reset temp mat 2 to z_size*z_size
	kf_mat_restruct(&kf->temp_mat2, kf->z_size, kf->z_size);
	mat_inv(&kf->temp_mat0, &kf->temp_mat2); // temp (H*P_pri*H^T + R)^-1 -> temp_mat2: z_size*z_size
	kf->mat_status = mat_mult(&kf->temp_mat1, &kf->temp_mat2, &kf->K); // K = H*P_pri*(H*P_pri*H^T + R)^-1 -> No.3 formula -> K: x_size*z_size

	/* update posterior estimation xhat */
	kf_mat_restruct(&kf->temp_vector0, kf->z_size, 1);
	kf_mat_restruct(&kf->temp_vector1, kf->z_size, 1);
	kf_mat_restruct(&kf->temp_vector2, kf->x_size, 1);
	mat_mult(&kf->H, &kf->xhat_pri, &kf->temp_vector0);  //temp H*xhat_pri -> temp_vector0: z_size*1
	mat_sub(&kf->z, &kf->temp_vector0, &kf->temp_vector1); // temp z-H*xhat_pri -> temp_vector1: z_size*1
	mat_mult(&kf->K, &kf->temp_vector1, &kf->temp_vector2); // temp K*(z-H*xhat_pri) -> temp_vector2: x_size*1
	kf->mat_status = mat_add(&kf->xhat_pri, &kf->temp_vector2, &kf->xhat); // xhat = xhat_pri + K*(z-H*xhat_pri) -> No.4 formula -> xhat: x_size*1

	/* update posterior est error covariance matrix */
	kf_mat_restruct(&kf->temp_mat0, kf->x_size, kf->x_size);
	kf_mat_restruct(&kf->temp_mat1, kf->x_size, kf->x_size);
	mat_mult(&kf->K, &kf->H, &kf->temp_mat0); // temp K*H -> temp_mat0: x_size*x_size
	mat_mult(&kf->temp_mat0, &kf->P_pri, &kf->temp_mat1); // temp K*H*P_pri -> temp_mat1: x_size*x_size
	kf->mat_status = mat_sub(&kf->P_pri, &kf->temp_mat1, &kf->P); // P = P_pri - K*H*P_pri -> No.5 formula -> P: x_size*x_size

	if(kf->mat_status == ARM_MATH_SUCCESS)
		return KF_OK;
	else
		return KF_ERR;
}

/**
  * @brief     apply kalman filter
  * @param[in] main structure of the kalman filter
  * @param[in] new input vector
  * @retval    None
  */
KFStatus_t kf_execute(KalmanFilter_t *kf, mat *input){
	/* update input value */
	kf_update_input(kf, input);
	/* apply kalman filter */
	if(kf_data_fusion(kf) == KF_OK)
		return KF_OK;
    else
    	return KF_ERR;
}

/**
  * @brief     deinit function of the kalman filter
  * @param[in] main structure of the kalman filter
  * @retval    None
  */
void kf_param_deinit(KalmanFilter_t *kf){
	/* release all of the memories set before*/
    vPortFree(kf->x.pData);
    vPortFree(kf->xhat.pData);
    vPortFree(kf->xhat_pri.pData);
    vPortFree(kf->u.pData);
    vPortFree(kf->z.pData);
    vPortFree(kf->A.pData);
    vPortFree(kf->B.pData);
    vPortFree(kf->H.pData);
    vPortFree(kf->Q.pData);
    vPortFree(kf->R.pData);
    vPortFree(kf->P.pData);
    vPortFree(kf->P_pri.pData);
    vPortFree(kf->K.pData);
    vPortFree(kf->temp_mat0.pData);
    vPortFree(kf->temp_mat1.pData);
    vPortFree(kf->temp_mat2.pData);
    vPortFree(kf->temp_mat3.pData);
    vPortFree(kf->temp_vector0.pData);
    vPortFree(kf->temp_vector1.pData);
    vPortFree(kf->temp_vector2.pData);
}



/* Below is the first order Kalman filter copied from robomaster community for reference */

/** *****************************************************************************************
  * @func    First Order Kalman Filter
  * @author  Liu heng
  * @version V1.0.0
  * @date    27-August-2013
  * @brief   The specific implementation of a one-dimensional Kalman filter. The implementation process is completely independent of hardware,
  *          can be directly invoked, and ported at will.
  *          When using, first define a Kalman pointer, then call kalmanCreate() to create a filter,
  *          each time sensor data is read, you can call KalmanFilter() to filter the data.
  *****************************************************************************************
  *                          Usage example                                                 *
  *          kalman p;                                                                     *
  *          float SensorData;                                                             *
  *          kalmanCreate(&p,20,200);                                                      *
  *          while(1)                                                                      *
  *          {                                                                             *
  *             SensorData = sensor();                                                     *
  *             SensorData = KalmanFilter(&p,SensorData);                                  *
  *             printf("%2.2f",SensorData);                                                *
  *          }                                                                             *
  *****************************************************************************************
  *          Reference parameters for the Kalman filter of MPU6050: Q:10 R:400             *
  *****************************************************************************************/

/**
  * @brief  Create a Kalman filter
  * @param  p:  filter
  *         T_Q: System noise covariance
  *         T_R: Measurement noise covariance
  *
  * @retval none
  */
void kalmanCreate(kalman_filter_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @brief  First Order Kalman filter
  * @param  p:  filter
  *         dat: Data to be filtered
  * @retval Filtered data
  */
float KalmanFilter(kalman_filter_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //state update
    p->X_last = p->X_now;
    return p->X_now;
}

#endif /*__KALMAN_FILTER_C__*/
