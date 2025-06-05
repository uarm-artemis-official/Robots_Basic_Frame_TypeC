#ifndef __MOTOR_TYPES_H
#define __MOTOR_TYPES_H

#include "stdint.h"

typedef enum {
    // Stop the motor immediately, WILL NOT respond to any command except START
    LK_MOTOR_IDLE = 0x80,
    // Stop the motor immediately but still respond to any command
    LK_MOTOR_STOP = 0x81,
    LK_MOTOR_START = 0x88,  // Reactivate the motor after sending KL_MOTOR_IDLE

    // Read feedback from motor [temp, torque current, speed, encoder pos]
    LK_MOTOR_READ_FB_DATA = 0x9C,

    // Read single loop feedback from motor (angle between [0, 360) in 0.01 degree steps).
    LK_MOTOR_READ_SL_FB = 0x94,

    // Reset the zero point of the motor (use sparingly to avoid ROM damage)
    LK_MOTOR_RESET_ZEROS = 0x19,
    // Similar to KL_MOTOR_RESET_ZEROS, requires testing
    KL_MOTOR_RESET_POS = 0x95,

    LK_CMD_TOR = 0xA1,  // Torque Close Loop Control, -2048 ~ 2048
    // Multiple Loop Angle Control, sending value 1000 ~= 1 degree
    LK_CMD_ML_ANGLE = 0xA3,
    LK_CMD_ML_ANGLE_WITH_SPEED = 0xA4,
    // Multiple Loop Speed Control, -2048 ~ 2048
    LK_CMD_SL_ANGLE_WITH_SPEED1 = 0xA5,
    LK_CMD_SL_ANGLE_WITH_SPEED = 0xA6,
    LK_CMD_INCREMENTAL_ANGLE = 0xA7
} LK_Motor_Command_t;

typedef enum {
    GM6020 = 0x1FF,
    M3508 = 0x200,
    M2006 = 0x1FF,
    MG4005E = 0x140,
} Motor_Tx_Stdid_t;

typedef struct {
    int16_t rx_angle;
    int16_t rx_rpm;
    int16_t rx_current;
    int16_t rx_temp;
} Motor_Feedback_t;

#endif