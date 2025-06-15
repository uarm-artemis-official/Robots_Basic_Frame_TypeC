#ifndef __UARM_DEFINES_H
#define __UARM_DEFINES_H

#ifdef __cpluspls
extern "C" {
#endif

/* =========================================================================
 * UARM MATH DEFINES
 * ====================================================================== */
#define DEGREE2RAD 0.0174533f
#define RAD2DEGEE 57.3f

#define MAX_WINDOW_SIZE 300

#define VAL_LIMIT(val, min, max)     \
    do {                             \
        if ((val) <= (min)) {        \
            (val) = (min);           \
        } else if ((val) >= (max)) { \
            (val) = (max);           \
        }                            \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)      \
    do {                                 \
        (val) = (angle) - (int) (angle); \
        (val) += (int) (angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do {                            \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

#ifdef __cpluspls
}
#endif

#endif