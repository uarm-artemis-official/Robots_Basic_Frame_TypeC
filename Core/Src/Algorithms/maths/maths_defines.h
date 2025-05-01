#ifndef __MATHS_CONFIG_H
#define __MATHS_CONFIG_H

/* =========================================================================
 * KALMAN FILTERS DEFINES
 * ====================================================================== */
/* define general declarations for gimbal task here */
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


/* =========================================================================
 * MATHS DEFINES
 * ====================================================================== */
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

#endif