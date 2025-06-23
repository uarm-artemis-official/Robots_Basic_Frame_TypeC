
// TODO: Rename to .hpp
#ifndef __SUBSYSTEMS_TYPES_H
#define __SUBSYSTEMS_TYPES_H

#include <array>
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
 * @Note  first 4 motors will use 0x200
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
    SWERVE_ZERO,
    DJI_GIMBAL,
    MOTORS_NONE,
} Motor_Config_t;

typedef enum { DJI, LK, UNKNOWN_MOTOR } Motor_Brand_t;

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
 * @Note  first 4 motors will use 0x200
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
    float prev_temp;
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
    UI_SEND,
    REF_INFO,
    PLAYER_COMMANDS,
    RC_INFO,
    RC_RAW,

    UC_PACK_IN,
    UC_PACK_OUT,
    AUTO_AIM,
    UART_OUT,
    GIMBAL_REL_ANGLES,

    // Referee System
    REFEREE_IN,
    REFEREE_OUT,
    COMMAND_GIMBAL,
    COMMAND_CHASSIS,
    COMMAND_SHOOT,
} Topic_Name_t;

// TODO: Find better solution to defining a testable QueueHandle_t.
#ifdef GTEST
typedef uint8_t* QueueHandle_t;
#endif

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
    uint8_t feedback[MAX_MOTOR_COUNT][8];
    Motor_CAN_ID_t can_ids[MAX_MOTOR_COUNT];
} MotorReadMessage_t;

typedef struct {
    float v_perp;
    float v_parallel;
    float wz;
    uint16_t command_bits;  // TODO: Implement and remove RC_INFO.
} ChassisCommandMessage_t;

typedef struct {
    float yaw;
    float pitch;
    uint32_t command_bits;  // TODO: Implement and remove AUTO_AIM topic.
} GimbalCommandMessage_t;

typedef struct {
    uint32_t command_bits;
    uint32_t extra_bits;
} ShootCommandMessage_t;

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

/* =========================================================================
 * REFEREE UI TYPES
 * ====================================================================== */
typedef enum { FIGURE_ID = 0, INFO_ID = 1, NUMBER_ID = 2 } referee_name_type_t;

typedef enum { UNKOWN = 0, RED_TEAM, BLUE_TEAM } robot_color_t;

/* used id define */
typedef enum {
    IDLE_ID = 0x0000,
    GAME_STAT_ID = 0x0001,
    GAME_RESULT_ID = 0x0002,
    GMAE_HP_ID = 0x0003,
    ROBOT_STAT_ID = 0x0201,
    POWER_HEAT_ID = 0x0202,
    SHOOT_ID = 0x0207,

    // Drawing UI interaction
    INTERA_UI_ID = 0x0301,
    INTERA_USER_DATA_ID = 0x0302,
} referee_id_t;

typedef struct __attribute__((__packed__)) {
    uint8_t winner;
} game_result_t;

/* define user structure here */
/* copy those from DJI referee manual */
typedef struct __attribute__((__packed__))  //0x0001
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef struct __attribute__((__packed__))  //0x0003
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

typedef struct __attribute__((__packed__))  //0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

typedef struct  //0x0202
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct __attribute__((__packed__))  //0x0207
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

/* For drawing the UI on the client */
typedef struct __attribute__((__packed__))  //0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];  //max length 113
} robot_interaction_data_t;

// Sub commands for UI interactions
typedef struct __attribute__((__packed__))  //0x0100
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

typedef struct __attribute__((__packed__))  //0x0101
{
    uint8_t figure_name[3];
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
} interaction_figure_t;

typedef struct __attribute__((__packed__))  //0x0102
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

typedef struct __attribute__((__packed__))  //0x0103
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_5_t;

typedef struct __attribute__((__packed__))  //0x0104
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_7_t;

typedef struct __attribute__((__packed__))  //0x0110
{
    interaction_figure_t figure_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

typedef struct __attribute__((__packed__))  //0x0302
{
    uint8_t data[30];  // max 30
} custom_robot_data_t;

/* main referee system struct */
typedef struct __attribute__((__packed__)) {
    uint8_t sof;  //0xA5
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
} frame_header_t;

// TODO: Re-enable later.
typedef enum {
    UI_TYPE_IDLE_ID = 0,
    /* General UI */
    UI_ROBOT_VAILD_INFO,
    UI_ROBOT_ACT_MODE,
    UI_ROBOT_LEVEL,
    UI_SUPCAP_PERCENT,

    /* Infantry only */
    UI_INFANTRY_MARK,

    /* Hero only */
    UI_HERO_MARK,
} referee_ui_type_t;

typedef enum {
    UI_IDLE_ID = 0x0000,  // No interaction
    // Sub cmd id
    SUB_UI_LAYER_DEL_ID = 0x0100,
    SUB_UI_DRAW_1_ID = 0x0101,
    SUB_UI_DRAW_2_ID = 0x0102,
    SUB_UI_DRAW_5_ID = 0x0103,
    SUB_UI_DRAW_7_ID = 0x0104,
    SUB_UI_EXT_CUSTOM_ID = 0x0110
} ref_ui_id_t;

typedef struct {
    uint32_t act_mode;
    uint32_t level;
    uint32_t super_cap_percent;
} ref_ui_info_t;

typedef struct {
    robot_interaction_data_t ui_intrect_data;
    robot_interaction_data_t ui_intrect_data_info;
    interaction_layer_delete_t ui_del_fig_data;
    interaction_figure_t ui_figure_data;
    interaction_figure_2_t ui_figure_draw_2_data;
    interaction_figure_5_t ui_figure_draw_5_data;
    interaction_figure_7_t ui_draw_marks_data;
    interaction_figure_7_t ui_draw_info_data;
    ext_client_custom_character_t ui_custom_data;

    uint8_t first_drawing_flag;
    uint8_t first_drawing_flag_info;  // For infantry marks
    uint8_t cur_sending_count;
    referee_ui_type_t cur_sending_id;
    ref_ui_info_t ui_info;
    uint8_t pack_seq;
} Referee_UI_t;

/* =========================================================================
 * RC COMM TYPES
 * ====================================================================== */
constexpr uint32_t DBUS_BUFFER_LENGTH = 18;
constexpr uint16_t CHANNEL_CENTER = 1024;
constexpr uint16_t MOUSE_MAX_SPEED = 15000;
constexpr uint16_t MAX_CHANNEL_VALUE = 660;

using Buffer = std::array<uint8_t, DBUS_BUFFER_LENGTH>;

enum class EKeyStatus {
    RELEASED = 0,        // key released
    RELEASED_TO_PRESS,   // key just pressed, rising edge
    PRESSED_TO_RELEASE,  // key just released, falling edge
    PRESSED              // key pressed
};

enum class ESwitchState {
    UNKNOWN = 0,
    UP = 1,
    MID = 3,
    DOWN = 2,
};

enum class EKeyBitIndex {
    W = 0x01 << 0,  // Also used for mouse clicks.
    S = 0x01 << 1,
    A = 0x01 << 2,
    D = 0x01 << 3,
    SHIFT = 0x01 << 4,
    CTRL = 0x01 << 5,
    Q = 0x01 << 6,
    E = 0x01 << 7,
    R = 0x01 << 8,
    F = 0x01 << 9,
    G = 0x01 << 10,
    Z = 0x01 << 11,
    X = 0x01 << 12,
    C = 0x01 << 13,
    V = 0x01 << 14,
    B = 0x01 << 15,
};

struct Controller {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    ESwitchState s1;
    ESwitchState s2;
    int16_t wheel;
};

/* pc mode */
struct KeyObject {
    EKeyStatus status;
    EKeyStatus pre_status;
    uint8_t status_count;
};

struct Keyboard {
    KeyObject W;
    KeyObject A;
    KeyObject S;
    KeyObject D;
    KeyObject Q;
    KeyObject E;
    KeyObject R;
    KeyObject V;
    KeyObject Ctrl;
    KeyObject F;
    KeyObject Shift;
    KeyObject G;
    KeyObject C;
    KeyObject B;

    uint16_t key_buffer;  // used to grab current key info
};

struct Mouse {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t click_l;
    uint8_t click_r;
    KeyObject left_click;
    KeyObject right_click;
};

struct PC {
    Mouse mouse;
    Keyboard keyboard;
};

/* =========================================================================
 * AMMO LID TYPES
 * ====================================================================== */
enum class EAmmoLidStatus {
    OPEN,
    CLOSED,
};

/* =========================================================================
 * PC COMM TYPES
 * ====================================================================== */
// PACK STRUCTS
// upper computer -> MCU
typedef struct {
    uint8_t target_num;
    float delta_yaw;
    float delta_pitch;
} UC_Auto_Aim_Pack_t;

typedef struct {
    uint8_t control_code;
    uint32_t sequence_number;
    uint32_t acknowledge;
} UC_Flow_Control_Pack_t;

// MCU -> upper computer
typedef struct {
    uint8_t robot_color;  // 0: red, 1: blue

    // imu data
    float pitch;
    float yaw;
    float accel_x;
    float accel_y;

    float wheel_rpm[4];  // from chassis wheels
} UC_Board_Data_Pack_t;

// OTHER STRUCTS
typedef union pack_checksum {
    uint32_t as_integer;
    uint8_t as_bytes[PACK_TRAILER_SIZE];
} UC_Checksum_t;

typedef struct {
    uint8_t header_id;
    uint8_t header_size;
    uint8_t data_size;
    uint8_t trailer_size;
} Pack_Metadata_t;

#endif