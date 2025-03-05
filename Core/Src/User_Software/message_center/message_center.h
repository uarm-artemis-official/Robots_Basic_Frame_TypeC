#ifndef PUBSUB_H
#define PUBSUB_H

#include "cmsis_os.h"
#include "stdint.h"
#include "stdlib.h"
#include "public_defines.h"


typedef enum Topic_Name_t {
	MOTOR_SET = 100,
	MOTOR_READ,
	MOTOR_IN,
	COMM_OUT,
	COMM_IN,
	IMU_READINGS,
	IMU_READY,
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
	int32_t motor_can_volts[MOTOR_TX_BUFFER_SIZE];
	uint32_t data_enable;
} MotorSetMessage_t;

void message_center_init();
BaseType_t get_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait);
BaseType_t peek_message(Topic_Name_t topic, void *data_ptr, int ticks_to_wait);
BaseType_t pub_message(Topic_Name_t topic, void *data_ptr);
BaseType_t pub_message_from_isr(Topic_Name_t topic, void *data_ptr, BaseType_t *will_context_switch);

#endif // !PUBSUB_H
