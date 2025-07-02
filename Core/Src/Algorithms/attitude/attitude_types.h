#ifndef __ATTITUDE_RESOLUTION_TYPES_H
#define __ATTITUDE_RESOLUTION_TYPES_H

#include "uarm_types.h"

/* =========================================================================
 * AHRS TYPES
 * ====================================================================== */
typedef struct {
    float ax;
    float ay;
    float az;

    float wx;
    float wy;
    float wz;

    float mx;
    float my;
    float mz;

    float roll;
    float pitch;
    float yaw;

} AhrsSensor_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

/* =========================================================================
 * MADGEWICK TYPES
 * ====================================================================== */
typedef struct {
    const int sampling_rate;
    const float beta;

    float q0;
    float q1;
    float q2;
    float q3;
} Madgewick_t;

#endif