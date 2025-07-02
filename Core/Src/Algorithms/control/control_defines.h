#ifndef __CONTROL_DEFINES_H
#define __CONTROL_DEFINES_H

/* =========================================================================
 * FEED FORWARD DEFINES 
 * ====================================================================== */
/* define general declarations for gimbal task here */
#define CHASSIS_FF_GAIN 100 //roughlt setting
#define GIMBAL_FF_GAIN 100
#define SHOOT_FF_GAIN 100


/* =========================================================================
 * PID DEFINES
 * ====================================================================== */
#define SINGLE_LOOP_PID_CONTROL 0
#define DUAL_LOOP_PID_CONTROL 1
#define SINGLE_LOOP_SHOOT_CONTROL 2


/* =========================================================================
 * RAMP DEFINES
 * ====================================================================== */
#define RAMP_GEN_DAFAULT     \
  {                          \
    .count = 0,              \
    .scale = 0,              \
    .out = 0,                \
    .init = &ramp_init,      \
    .calc = &ramp_calculate, \
  }

#endif