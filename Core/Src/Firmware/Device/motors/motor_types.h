#ifndef __MOTOR_TYPES_H
#define __MOTOR_TYPES_H

#include "stdint.h"

typedef enum {
    LK_MOTOR_IDLE =
        0x80,  // Stop the motor immediately, WILL NOT respond to any command except START
    LK_MOTOR_STOP =
        0x81,  // Stop the motor immediately but still respond to any command
    LK_MOTOR_START = 0x88,  // Reactivate the motor after sending KL_MOTOR_IDLE

    LK_MOTOR_READ_FB_DATA =
        0x9C,  // Read feedback from motor [temp, torque current, speed, encoder pos]
    LK_MOTOR_RESET_ZEROS =
        0x19,  // Reset the zero point of the motor (use sparingly to avoid ROM damage)
    KL_MOTOR_RESET_POS =
        0x95,  // Similar to KL_MOTOR_RESET_ZEROS, requires testing

    LK_CMD_TOR = 0xA1,  // Torque Close Loop Control, -2048 ~ 2048
    LK_CMD_ML_ANGLE =
        0xA3,  // Multiple Loop Angle Control, sending value 10 ~= 1 degree
    LK_CMD_ML_SPEED = 0xA7  // Multiple Loop Speed Control, -2048 ~ 2048
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