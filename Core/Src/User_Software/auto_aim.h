/*******************************************************************************
* @file           : auto_aim.h
* @brief          : auto aim
* @created time	  : Oct, 2023
* @modified time  : Mar, 2024
* @authors        : Haoran Qi, James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __AUTO_AIM_H__
#define __AUTO_AIM_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"

// PACKET STRUCTS AND DEFINITIONS
// All packet sizes are in bytes.
#define UC_AUTO_AIM_PACK_HEADER 0x99
#define UC_AUTO_AIM_PACK_SIZE 20
typedef struct{
	uint8_t header;
	uint8_t salt;
	uint16_t header_checksum;

	uint8_t target_num;
	float delta_yaw;
	float delta_pitch;

	uint32_t checksum;
} UC_Auto_Aim_Pack_t; //receive packet from upper computer


#define UC_IMU_DATA_PACK_HEADER 0x5D
#define UC_IMU_DATA_PACK_SIZE 44
typedef struct{
	uint8_t header;
	uint8_t salt;
	uint16_t header_checksum;

	uint8_t robot_color; // 0: red, 1: blue
	uint8_t task_mode; // no use

	// imu data
	float pitch;
	float yaw;
	float accel_x;
	float accel_y;

	float wheel_rpm[4]; // from chassis wheels

	uint32_t checksum;
} UC_IMU_Data_Pack_t;


#define UC_BOARD_RESPONSE_PACK_HEADER 0x11
#define UC_BOARD_RESPONSE_PACK_SIZE 12
typedef struct {
	uint8_t header;
	uint8_t salt;
	uint16_t header_checksum;
	/* Response code meanings
	 *  0 - packet received with no errors.
	 *  1 - packet received with errors.
	 */
	uint8_t response_code;

	uint32_t checksum;
} UC_Board_Response_Pack_t;


#define UC_PC_COMMAND_PACK_HEADER 0x22
#define UC_PC_COMMAND_PACK_SIZE 60
typedef struct {
	uint8_t header;
	uint8_t salt;
	uint16_t header_checksum;

	uint8_t command_code;
	uint32_t data[12];

	uint32_t checksum;
} UC_PC_Command_Pack_t;


// EXTERNS
extern UART_HandleTypeDef huart1;
extern QueueHandle_t UC_Pack_Queue;


// GLOBALS
#define UC_PACK_BUFFER_SIZE 64 // Maximum pack size (in bytes).
#define UC_HEADER_SIZE 4       // Header size for all packs (in bytes).
uint8_t uc_pack_buffer[UC_PACK_BUFFER_SIZE];
uint8_t uc_valid_header;


// INIT FUNCTIONS
void uc_auto_aim_pack_init(UC_Auto_Aim_Pack_t *uc_pack);
void uc_imu_data_pack_init(UC_IMU_Data_Pack_t *uc_tx_pack);
void uc_board_response_pack_init(UC_Board_Response_Pack_t *uc_response_pack);
void uc_pc_command_pack_init(UC_PC_Command_Pack_t *uc_pc_command_pack);


// UC COMMUNICATION FUNCTIONS
void uc_receive_pack();
uint8_t uc_send_pack(void* packet, uint16_t packet_size);

uint32_t calculate_checksum(void *data, size_t size);
uint16_t uc_get_pack_size(uint8_t *pack_ptr);
uint8_t uc_get_header(uint8_t *pack_ptr);
void uc_on_RxCplt();

#endif /*__AUTO_AIM_H__*/
