#ifndef __DEVICE_TYPES_H
#define __DEVICE_TYPES_H

#include "control_types.h"
#include "public_defines.h"
#include "stdint.h"

/* =========================================================================
 * BUZZER TYPES
 * ====================================================================== */
typedef struct{
	uint32_t buzzer_tick;
	uint32_t times_tick;

	uint8_t buzz_times;
}Buzzer_t;

/* =========================================================================
 * DWT TYPES
 * ====================================================================== */
typedef struct
{
    uint32_t us;
} DWTTime_t;

/* =========================================================================
 * MOTOR TYPES
 * ====================================================================== */
typedef enum {
    LK_MOTOR_IDLE = 0x80,       // Stop the motor immediately, WILL NOT respond to any command except START
    LK_MOTOR_STOP = 0x81,       // Stop the motor immediately but still respond to any command
    LK_MOTOR_START = 0x88,      // Reactivate the motor after sending KL_MOTOR_IDLE

    LK_MOTOR_READ_FB_DATA = 0x9C, // Read feedback from motor [temp, torque current, speed, encoder pos]
    LK_MOTOR_RESET_ZEROS = 0x19,  // Reset the zero point of the motor (use sparingly to avoid ROM damage)
    KL_MOTOR_RESET_POS = 0x95,    // Similar to KL_MOTOR_RESET_ZEROS, requires testing

    LK_CMD_TOR = 0xA1,          // Torque Close Loop Control, -2048 ~ 2048
    LK_CMD_ML_ANGLE = 0xA3,     // Multiple Loop Angle Control, sending value 1000 ~= 1 degree
    LK_CMD_ML_SPEED = 0xA7      // Multiple Loop Speed Control, -2048 ~ 2048
} LK_Motor_Command_t;



/*
 * @brief Structure for all motors installed
 * @Note  first 4 motors will use 0x200, last 4 motors will use 0x1FF
 * 		    motor_data[0]: chassis 	3508
 * 			motor_data[1]: chassis 	3508
 *			motor_data[2]: chassis 	3508
 *			motor_data[3]: chassis  3508
 *			motor_data[4]: gimbal 	6020 pitch
 *			motor_data[5]: gimbal 	6020 yaw
 *			motor_data[6]: magazine 2006
 * */

/**
  * @brief basic gimbal struct
  */
typedef struct {
	uint32_t stdid;
	PID_t f_pid; //first pid handler for single-loop control
	PID_t s_pid; //second pid handler for dual-loop control
	FeedForward_t ff;
} Motor_Info;


typedef struct {
	Motor_Info motor_info;
	Motor_Feedback_t motor_feedback;
	//Data need to sent to Motor
	int32_t tx_data;
} Motor_t;

typedef struct {
	uint32_t send_id;
	LK_Motor_Command_t motor_cmd;
	Motor_Feedback_t motor_feedback;
	int32_t tx_data; // Data need to sent to LK Motor
} LK_Motor_t;

/*
 * @brief Structure for all motors installed
 * @Note  first 4 motors will use 0x200, last 4 motors will use 0x1FF
 * 		    motor_data[0]: chassis 	3508/shoot fric left  3508
 * 			motor_data[1]: chassis 	3508/shoot fric right 3508
 *			motor_data[2]: chassis 	3508/shoot magazine 3508
 *			motor_data[3]: chassis  3508
 *			motor_data[4]: gimbal 	6020 pitch
 *			motor_data[5]: gimbal 	6020 yaw
 *			motor_data[6]: magazine 2006
 * */

#endif