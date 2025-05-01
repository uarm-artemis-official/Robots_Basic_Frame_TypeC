#ifndef __ATTITUDE_RESOLUTION_TYPES_H
#define __ATTITUDE_RESOLUTION_TYPES_H

#include "stdint.h"

/* =========================================================================
 * AHRS TYPES
 * ====================================================================== */
typedef struct
{
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

}AhrsSensor_t;

typedef struct
{
  float roll;
  float pitch;
  float yaw;

  uint32_t timestamp;
}Attitude_t;

#endif 