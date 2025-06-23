#ifndef __SUBSYSTEMS_DEFINES_H
#define __SUBSYSTEMS_DEFINES_H

/* =========================================================================
 * MOTOR DEFINES
 * ====================================================================== */
#define MOTOR_COUNT 8  // RM motor 0-7,
#define LK_MOTOR_COUNT 4
#define MAX_MOTOR_COUNT 8

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

// These are used for indexing the motor feedback array from MOTOR_READ topic.
// Formula: (CAN ID) = (Chassis Motor CAN Std. IDs) - CAN_RX_ID_START.
#define CHASSIS_WHEEL1_CAN_ID 0
#define CHASSIS_WHEEL2_CAN_ID 1
#define CHASSIS_WHEEL3_CAN_ID 2
#define CHASSIS_WHEEL4_CAN_ID 3

// These are used for indexing the motor feedback array from MOTOR_READ topic.
// Formula: (CAN ID) = (Gimbal Motor CAN Std. IDs) - CAN_RX_ID_START.
#define SHOOT_LEFT_FRIC_CAN_ID 0
#define SHOOT_RIGHT_FRIC_CAN_ID 1
#define SHOOT_LOADER_CAN_ID 6
#define GIMBAL_YAW_CAN_ID 4
#define GIMBAL_PITCH_CAN_ID 5

/* =========================================================================
 * IMU DEFINES
 * ====================================================================== */

/* =========================================================================
 * EVENT CENTER DEFINES
 * ====================================================================== */
// Number of sync events that can occur at the same time.
// A sync event group has one event group that can allows up to 24 different tasks
// (listed in Task_Sync_t) to "sync" to one particular event (Sync_Event_t).
#define NUM_SYNC_GROUPS 5

/* =========================================================================
 * CAN COMM DEFINES
 * ====================================================================== */
#define CAN_COMM_QUEUE_SIZE 5

/* =========================================================================
 * REFEREE UI DEFINES
 * ====================================================================== */
#define SOF_ID 0xA5                        //fixed sof value
#define HEADER_LEN sizeof(frame_header_t)  // 5
#define CMD_LEN 2                          //cmd_id bytes
#define CRC_LEN 2                          //crc16 bytes
#define MAX_REF_BUFFER_SIZE 256
#define MAX_REF_TX_DATA_LEN 128
#define MAX_REF_RX_DATA_LEN 41

#define UI_HEADER_LEN 6
#define UI_SEVEN_DRAWS_LEN 105  // 7 * sizeof(interaction_figure_t)=15
#define UI_HERO_MARK_LEN UI_HEADER_LEN + UI_SEVEN_DRAWS_LEN
#define UI_INFANTRY_MARK_LEN UI_HEADER_LEN + UI_SEVEN_DRAWS_LEN
#define UI_STRING_DRAWS_LEN 45  // 15 + 30
#define UI_ACT_MODE_LEN UI_HEADER_LEN + UI_STRING_DRAWS_LEN
#define UI_ROBOT_VAILD_INFO_LEN UI_HEADER_LEN + UI_SEVEN_DRAWS_LEN

/* =========================================================================
 * PC COMM DEFINES
 * ====================================================================== */
// PACK HEADERS
#define UC_AUTO_AIM_HEADER 0x01
#define UC_BOARD_DATA_HEADER 0x02
#define UC_FLOW_CONTROL_HEADER 0x04

// PACK DEFINITIONS
#define MAX_PACK_BUFFER_SIZE 64  // Measured in bytes.
#define PACK_HEADER_SIZE 4
#define PACK_TRAILER_SIZE 4

#endif