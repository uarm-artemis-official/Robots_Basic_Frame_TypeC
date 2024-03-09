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

// PACKET STRUCTS AND DEFINITIONS
// All packet sizes are in bytes.

#define UC_RECV_PACK_HEADER 0x99
#define UC_RECV_PACK_SIZE 16
typedef struct{
	uint8_t header;
	uint8_t target_num;
	float delta_yaw;
	float delta_pitch;
	uint32_t checksum;
} UC_Recv_Pack_t; //receive packet from upper computer

#define UC_SEND_PACK_HEADER 0x5D
#define UC_SEND_PACK_SIZE 40
typedef struct{
	uint8_t header;
	uint8_t robot_color; // 0: red, 1: blue
	uint8_t task_mode; // no use

	// imu data
	float pitch;
	float yaw;
	float accel_x;
	float accel_y;

	float wheel_rpm[4]; // from chassis wheels
	uint32_t checksum;
} UC_Send_Pack_t;

#define UC_RESPONSE_PACK_HEADER 0x11
#define UC_RESPONSE_PACK_SIZE 8
typedef struct {
	uint8_t header;
	/* Response code meanings
	 *  0 - packet received with no errors.
	 *  1 - packet received with errors.
	 */
	uint8_t response_code;
	uint32_t checksum;
} UC_Response_Pack_t;


extern UART_HandleTypeDef huart1;
uint8_t uc_input_buffer[UC_RECV_PACK_SIZE];
UC_Recv_Pack_t uc_rx_pack;


// INIT FUNCTIONS
void uc_rx_pack_init(UC_Recv_Pack_t *uc_pack);
void uc_tx_pack_init(UC_Send_Pack_t *uc_tx_pack);
void uc_response_pack_init(UC_Response_Pack_t *uc_response_pack);

uint32_t calculate_checksum(void *data, size_t size);

// UC COMMUNICATION FUNCTIONS
void uc_receive_packet();
uint8_t uc_send_packet(void* packet, uint16_t packet_size);

#endif /*__AUTO_AIM_H__*/
