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

// Auto-aim externals
extern UART_HandleTypeDef huart1;
#define UC_HUART huart1


// PACK DEFINITIONS
// All the following definitions are measured in bytes.
#define UC_PACK_SIZE 64
#define UC_PACK_HEADER_SIZE 4
#define UC_PACK_DATA_SIZE 56
#define UC_PACK_TRAILER_SIZE 4


// PACK HEADERS
#define UC_AUTO_AIM_HEADER 0x01
#define UC_BOARD_DATA_HEADER 0x02
#define UC_FLOW_CONTROL_HEADER 0x04


// PACK STRUCTS
#define UC_AUTO_AIM_DATA_SIZE 12
typedef struct {
	uint8_t target_num;
	float delta_yaw;
	float delta_pitch;
} UC_Auto_Aim_Pack_t; //receive packet from upper computer


#define UC_BOARD_DATA_DATA_SIZE 36
typedef struct {
	uint8_t robot_color; // 0: red, 1: blue

	// imu data
	float pitch;
	float yaw;
	float accel_x;
	float accel_y;

	float wheel_rpm[4]; // from chassis wheels
} UC_Board_Data_Pack_t;


#define UC_FLOW_CONTROL_DATA_SIZE 12
typedef struct {
	uint8_t control_code;
	uint32_t sequence_number;
	uint32_t acknowledge;
} UC_Flow_Control_Pack_t;


// OTHER STRUCTS
typedef union pack_checksum {
	uint32_t as_integer;
	uint8_t as_bytes[UC_PACK_TRAILER_SIZE];
} UC_Checksum_t;


// INIT FUNCTIONS
void uc_auto_aim_pack_init(UC_Auto_Aim_Pack_t *pack);
void uc_board_data_pack_init(UC_Board_Data_Pack_t *pack);
void uc_flow_control_pack_init(UC_Flow_Control_Pack_t *pack);

UC_Checksum_t calculate_checksum(void *data, size_t size);

/*
 * Checks if the header in uc_pack_input_buffer is recognized header.
 * All recognized headers are defined under PACK HEADERS macros above.
 *
 * RETURNS:
 *  1 - if the header is a recognized header.
 *  0 - otherwise.
 */
uint8_t is_valid_header();

// UC COMMUNICATION FUNCTIONS
void uc_receive(uint8_t *input_buffer, uint16_t size);
uint8_t uc_send_board_data(UC_Board_Data_Pack_t *pack);
uint8_t uc_send(void* pack);
void uc_stop_receive();

#endif /*__AUTO_AIM_H__*/
