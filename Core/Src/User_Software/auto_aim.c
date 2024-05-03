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


void uc_auto_aim_pack_init(UC_Auto_Aim_Pack_t *uc_rx_pack){
	uc_rx_pack->target_num      = 0;
	uc_rx_pack->delta_yaw       = 0.0f;
	uc_rx_pack->delta_pitch     = 0.0f;
}


void uc_board_data_pack_init(UC_Board_Data_Pack_t *uc_tx_pack){
	uc_tx_pack->robot_color     = 0;
	uc_tx_pack->yaw             = 0.0f;
	uc_tx_pack->pitch           = 0.0f;
	uc_tx_pack->accel_x         = 0;
	uc_tx_pack->accel_y         = 0;
	uc_tx_pack->wheel_rpm[0]    = 0.0f;
	uc_tx_pack->wheel_rpm[1]    = 0.0f;
	uc_tx_pack->wheel_rpm[2]    = 0.0f;
	uc_tx_pack->wheel_rpm[3]    = 0.0f;
}


void uc_flow_control_pack_init(UC_Flow_Control_Pack_t *pack) {
	pack->control_code = 0;
	pack->sequence_number = 0;
	pack->acknowledge = 0;
}


UC_Checksum_t calculate_checksum(void *buffer_ptr, size_t buffer_size) {
	/*
	 * This function assumes the last 4 bytes represent the pack checksum sent from UC,
	 * therefore, they are ignored during checksum calculation (unless the checksum will
	 * always be different).
	 *
	 * ALGORITHM:
	 *  - Separates bytes of uc_pack_input_buffer into 32 bit integers.
	 *  - Add integers together.
	 *  - Mod sum by 1000000007 (arbitrary prime number).
	 *
	 */
	const uint32_t MOD = 1000000007;
	uint32_t* data = (uint32_t*) buffer_ptr;
	uint32_t running_checksum = 0;

	for (size_t i = 0; i < buffer_size / 4 - 1; i++){
		running_checksum = (running_checksum + (data[i] % MOD)) % MOD;
	}

	UC_Checksum_t checksum_union;
	checksum_union.as_integer = running_checksum;
	return checksum_union;
}


void uc_receive(uint8_t *input_buffer, uint16_t size) {
	/*
	 * Receives packs from mini-PC through UART with DMA.
	 *
	 * Data is first received in uc_input_buffer.
	 * If there are no errors in the received data, then the data
	 * is copied to uc_rx_pack to be used.
	 *
	 * Check uc_on_RxCplt() for error detection.
	 */
	HAL_UART_Receive_DMA(&UC_HUART, input_buffer, size);
}


uint8_t uc_send_board_data(UC_Board_Data_Pack_t *board_data_pack) {
	uint8_t pack_bytes[UC_PACK_SIZE];
	pack_bytes[0] = UC_BOARD_DATA_HEADER;
	memcpy(pack_bytes + UC_PACK_HEADER_SIZE, board_data_pack, UC_BOARD_DATA_DATA_SIZE);

	UC_Checksum_t checksum = calculate_checksum(pack_bytes, UC_PACK_SIZE);

	memcpy(pack_bytes + UC_PACK_SIZE - UC_PACK_TRAILER_SIZE, checksum.as_bytes, UC_PACK_TRAILER_SIZE);

	uint8_t response = uc_send(pack_bytes);
}


uint8_t uc_send(void* pack) {
	/*
	 * Send any pack to mini-PC through UART2 (huart1).
	 * Data is always sent in LSB order.
	 * If pack is pointing to a struct, top fields are sent first and bottom
	 * fields are sent last. Make sure you understand how fields are padded
	 * in a struct before using uc_send_pack.
	 *
	 * PARAMETERS:
	 *  - pack:      Pointer to continuous pack data.
	 *  - pack_size: Size of the pack (in bytes).
	 *
	 *  RETURN:
	 * 	  0 -> Transmission was okay.
	 * 	  1 -> Otherwise.
	 */
	// Data is sent in LSB order with top fields bottom fields in UC_Send_Pack_t.
	HAL_StatusTypeDef status = HAL_UART_Transmit(&UC_HUART, pack, UC_PACK_SIZE, 100);
	if (status == HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}


void uc_stop_receive() {
	HAL_UART_AbortReceive(&UC_HUART);
}


uint8_t is_valid_header(uint8_t *input_buffer) {
	// All valid packs will have the header as the first byte in the input buffer.
	// Check under PACK HEADER in auto_aim.h to see macros defining recognized headers.
	switch(input_buffer[0]) {
	case 0x01:
	case 0x02:
	case 0x04:
		return 1;
	default:
		return 0;
	}
}


#endif /*__AUTO_AIM_C__*/
