#ifndef __SUBSYSTEMS_TYPES_H
#define __SUBSYSTEMS_TYPES_H

#include "attitude_types.h"
#include "motor_types.h"
#include "subsystems_defines.h"
#include "uarm_os.h"
#include "uarm_types.h"

/* =========================================================================
 * MOTOR TYPES
 * ====================================================================== */
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

typedef enum {
    // DJI CHASSIS
    CHASSIS_WHEEL1 = 0x201,
    CHASSIS_WHEEL2 = 0x202,
    CHASSIS_WHEEL3 = 0x203,
    CHASSIS_WHEEL4 = 0x204,

    // SWERVE
    SWERVE_STEER_MOTOR1 = 0x141,
    SWERVE_STEER_MOTOR2 = 0x142,
    SWERVE_STEER_MOTOR3 = 0x143,
    SWERVE_STEER_MOTOR4 = 0x144,

    // SHOOT
    SHOOT_LEFT_FRIC = 0x201,
    SHOOT_RIGHT_FRIC = 0x202,
    SHOOT_LOADER = 0x207,

    // GIMBAL
    GIMBAL_YAW = 0x205,
    GIMBAL_PITCH = 0x206,
} Motor_CAN_ID_t;

typedef enum {
    DJI_CHASSIS,
    SWERVE,
    DJI_GIMBAL,
    MOTORS_NONE,
} Motor_Config_t;

typedef enum {
    DJI,
    LK,
} Motor_Brand_t;

typedef struct {
    // feedback_id is the same as stdid used to send commands to LK motors.
    uint32_t feedback_id;
    int32_t tx_data;
    Motor_Brand_t brand;
} Generic_Motor_t;

typedef struct {
    Generic_Motor_t motors[MAX_MOTOR_COUNT];
    Motor_Config_t config;
} System_Motors_t;

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

/* =========================================================================
 * IMU TYPES
 * ====================================================================== */
typedef enum { NORMAL = 1, ABNORMAL } IMU_temp_status;

typedef enum {
    GAM_MODE = 0,  // 9 axis imu
    GA_MODE        // 6 axis mpu
} IMU_mode_t;

typedef struct {
    float temp;
    uint32_t sample_time;

    IMU_temp_status temp_status;
    IMU_mode_t imu_mode;
    AhrsSensor_t ahrs_sensor;  //for ahrs sensor - processed data
} IMU_t;

/* =========================================================================
 * MESSAGE CENTER TYPES
 * ====================================================================== */
typedef enum Topic_Name_t {
    MOTOR_SET = 100,
    MOTOR_READ,
    MOTOR_IN,
    COMM_OUT,
    COMM_IN,
    IMU_READINGS,
    //	IMU_READY,
    REF_INFO,
    PLAYER_COMMANDS,
    RC_INFO,
    RC_RAW,
    REFEREE_INFO,
    UC_PACK_IN,
    AUTO_AIM,
    UART_OUT,

    // Gimbal -> Chassis
    GIMBAL_REL_ANGLES,
} Topic_Name_t;

typedef struct Topic_Handle_t {
    Topic_Name_t name;
    uint8_t item_size;
    uint8_t queue_length;
    QueueHandle_t queue_handle;
} Topic_Handle_t;

typedef struct {
    uint32_t topic_name;
    uint8_t data[8];
} CANCommMessage_t;

typedef struct {
    /*
	 * modes[0] - BoardMode_t
	 * modes[1] - BoardActMode_t
	 * modes[2] - ShootActMode_t
	 */
    uint8_t modes[3];

    /*
	 * channels[0-3] - Have info from RC on Chassis.
	 * channels[0-1] - Have info from Chassis on Gimbal.
	 * Gimbal is not given channels[2] and channels[3] because it does not need channels[0] and
	 * channels[1] for calculations. This allows RC info to be transmitted in one CAN frame (8 bytes).
	 */
    int16_t channels[4];
} RCInfoMessage_t;

typedef struct {
    int32_t motor_can_volts[MAX_MOTOR_COUNT];
    Motor_CAN_ID_t can_ids[MAX_MOTOR_COUNT];
} MotorSetMessage_t;

typedef struct {
    Motor_Feedback_t feedback[MAX_MOTOR_COUNT];
    Motor_CAN_ID_t can_ids[MAX_MOTOR_COUNT];
} MotorReadMessage_t;

/* =========================================================================
 * DEBUG TYPES
 * ====================================================================== */
typedef enum { GIMBAL_BOARD = 1, CHASSIS_BOARD = 2 } BoardStatus_t;

typedef enum {
    BLUE,
    RED,
    GREEN,
} Board_LED_t;

typedef enum {
    ON,
    OFF,
} Board_LED_State_t;

/* =========================================================================
 * EVENT CENTER TYPES
 * ====================================================================== */
// There is a maximum of 24 bits for our configuration for one event group.
// The other 8 bits are reserved for other use.
typedef enum Event_t { IMU_READY = 1 << 0 } Event_t;

typedef uint32_t UARM_Events_t;

typedef enum Task_Sync_t {
    TS_CHASSIS = 1 << 1,
    TS_GIMBAL = 1 << 2,
    TS_SHOOT = 1 << 3,
    TS_COMM = 1 << 4,
    TS_CONTROL = 1 << 5,
    TS_IMU = 1 << 6,
    TS_PC_UART = 1 << 7,
    TS_REFEREE = 1 << 8,
    TS_TIMER = 1 << 9,
    TS_WATCHDOG = 1 << 10,
} Task_Sync_t;

typedef enum Sync_Event_t {
    None,
} Sync_Event_t;

#endif