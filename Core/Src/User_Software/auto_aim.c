/*******************************************************************************
* @file           : auto_aim.c
* @brief          : This is the new uto aim arch based on rm_vision python
* @created time	  : Oct, 2023
* @modified time  : Mar, 2024
* @authors        : Haoran Qi, James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __AUTO_AIM_C__
#define __AUTO_AIM_C__


#include "auto_aim.h"


void uc_rx_pack_init(UC_Recv_Pack_t *uc_rx_pack){
	uc_rx_pack->header      = UC_RECV_PACK_HEADER;
	uc_rx_pack->target_num  = 0;
	uc_rx_pack->delta_yaw   = 0.0f;
	uc_rx_pack->delta_pitch = 0.0f;
	uc_rx_pack->checksum    = -1;
}


void uc_tx_pack_init(UC_Send_Pack_t *uc_tx_pack){
	uc_tx_pack->header       = UC_SEND_PACK_HEADER;
	uc_tx_pack->robot_color  = 0;
	uc_tx_pack->yaw          = 0.0f;
	uc_tx_pack->pitch        = 0.0f;
	uc_tx_pack->accel_x      = 0;
	uc_tx_pack->accel_y      = 0;
	uc_tx_pack->wheel_rpm[0] = 0.0f;
	uc_tx_pack->wheel_rpm[1] = 0.0f;
	uc_tx_pack->wheel_rpm[2] = 0.0f;
	uc_tx_pack->wheel_rpm[3] = 0.0f;
	uc_tx_pack->checksum     = -1;
}


void uc_response_pack_init(UC_Response_Pack_t *uc_response_pack) {
	uc_response_pack->header        = UC_RESPONSE_PACK_HEADER;
	uc_response_pack->response_code = -1;
	uc_response_pack->checksum      = -1;
}


uint32_t calculate_checksum(void *packet, size_t packet_size) {
	const uint32_t MOD = 1000000007;
	uint32_t* data = (uint32_t*) packet;
	uint32_t checksum = 0;

	// Calculate checksum only for data portion of a packet.
	// Assumes that the packet checksum is uint32_t and is the last
	// field of the packet struct.
	for (size_t i = 0; i < packet_size / 4 - 1; i++){
		checksum = (checksum + (data[i] % MOD)) % MOD;
	}
	return checksum;
}


void uc_receive_packet() {
	HAL_UART_Receive_DMA(&huart1, uc_input_buffer, UC_RECV_PACK_SIZE);
}


uint8_t uc_send_packet(void* packet, uint16_t packet_size) {
	// Data is sent in LSB order with top fields bottom fields in UC_Send_Pack_t.
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, packet, packet_size, 200);
	if (status == HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}


#endif /*__AUTO_AIM_C__*/
