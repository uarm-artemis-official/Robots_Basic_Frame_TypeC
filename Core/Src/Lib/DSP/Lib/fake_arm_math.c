
#include "fake_arm_math.h"

/**
 * @brief  Floating-point matrix initialization.
 * @param[in,out] S         points to an instance of the floating-point matrix structure.
 * @param[in]     nRows     number of rows in the matrix.
 * @param[in]     nColumns  number of columns in the matrix.
 * @param[in]     pData     points to the matrix data array.
 */
void arm_mat_init_f32(
    arm_matrix_instance_f32 *S,
    uint16_t nRows,
    uint16_t nColumns,
    float32_t *pData)
{
}

/**
 * @brief Floating-point matrix addition.
 * @param[in]  pSrcA  points to the first input matrix structure
 * @param[in]  pSrcB  points to the second input matrix structure
 * @param[out] pDst   points to output matrix structure
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */
arm_status arm_mat_add_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst)
{
    return ARM_MATH_SUCCESS;
}

/**
 * @brief Floating-point matrix subtraction
 * @param[in]  pSrcA  points to the first input matrix structure
 * @param[in]  pSrcB  points to the second input matrix structure
 * @param[out] pDst   points to output matrix structure
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */
arm_status arm_mat_sub_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst)
{
    return ARM_MATH_SUCCESS;
}

/**
 * @brief Floating-point matrix multiplication
 * @param[in]  pSrcA  points to the first input matrix structure
 * @param[in]  pSrcB  points to the second input matrix structure
 * @param[out] pDst   points to output matrix structure
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */
arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 *pSrcA,
    const arm_matrix_instance_f32 *pSrcB,
    arm_matrix_instance_f32 *pDst)
{
    return ARM_MATH_SUCCESS;
}

/**
 * @brief Floating-point matrix transpose.
 * @param[in]  pSrc  points to the input matrix
 * @param[out] pDst  points to the output matrix
 * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
 * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 */
arm_status arm_mat_trans_f32(
    const arm_matrix_instance_f32 *pSrc,
    arm_matrix_instance_f32 *pDst)
{
    return ARM_MATH_SUCCESS;
}

/**
 * @brief Floating-point matrix inverse.
 * @param[in]  src   points to the instance of the input floating-point matrix structure.
 * @param[out] dst   points to the instance of the output floating-point matrix structure.
 * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not match.
 * If the input matrix is singular (does not have an inverse), then the algorithm terminates and returns error status ARM_MATH_SINGULAR.
 */
arm_status arm_mat_inverse_f32(
    const arm_matrix_instance_f32 *src,
    arm_matrix_instance_f32 *dst)
{
    return ARM_MATH_SUCCESS;
}