#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

/* =========================================================================
 * DWT DEFINES
 * ====================================================================== */
#define SYSTEM_CORE_FREQ 168 //MHz

/* =========================================================================
 * MOTOR DEFINES
 * ====================================================================== */
#define MOTOR_COUNT 8 // RM motor 0-7,
#define LK_MOTOR_COUNT 4

#define CAN_RX_ID_START 0x201
#define MOTOR_3508_STDID 0x200
#define MOTOR_6020_STDID 0x1FF
#define MOTOR_2006_STDID 0x1FF

#define CHASSIS_MOTOR_MAX_DELTA 1000
#define GIMBAL_MOTOR_MAX_DELTA 1000
#define CHASSIS_MOTOR_PID_MAX_OUT 3500
#define GIMBAL_MOTOR_PID_MAX_OUT 2000
#define FRIC_MOTOR_PID_MAX_VALUE 2000
#define MAG_MOTOR_PID_MAX_VALUE 10000

/* KL Motor Command Defines */
#define LK_MOTOR_TX_STDID 0x140
#define LK_MOTOR_RX_STDID 0x280

#define LK_TOR_MAX 2048
#define LK_ANGLE_MAX (360 * 1000)
#define LK_SPEED_MAX 2048

#endif