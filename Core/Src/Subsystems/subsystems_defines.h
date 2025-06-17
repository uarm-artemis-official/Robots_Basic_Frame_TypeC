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
#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}

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
#define UI_HEADER_LEN 6
#define UI_SEVEN_DRAWS_LEN 105  // 7 * sizeof(interaction_figure_t)=15
#define UI_HERO_MARK_LEN UI_HEADER_LEN + UI_SEVEN_DRAWS_LEN
#define UI_INFANTRY_MARK_LEN UI_HEADER_LEN + UI_SEVEN_DRAWS_LEN
#define UI_STRING_DRAWS_LEN 45  // 15 + 30
#define UI_ACT_MODE_LEN UI_HEADER_LEN + UI_STRING_DRAWS_LEN

#endif