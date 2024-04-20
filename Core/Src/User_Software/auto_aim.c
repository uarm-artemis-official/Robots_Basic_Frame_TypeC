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
	uc_rx_pack->header          = UC_AUTO_AIM_PACK_HEADER;
	uc_rx_pack->salt            = -1;
	uc_rx_pack->header_checksum = -1;
	uc_rx_pack->target_num      = 0;
	uc_rx_pack->delta_yaw       = 0.0f;
	uc_rx_pack->delta_pitch     = 0.0f;
	uc_rx_pack->checksum        = -1;
}


void uc_imu_data_pack_init(UC_IMU_Data_Pack_t *uc_tx_pack){
	uc_tx_pack->header          = UC_IMU_DATA_PACK_HEADER;
	uc_tx_pack->salt            = -1;
	uc_tx_pack->header_checksum = -1;
	uc_tx_pack->robot_color     = 0;
	uc_tx_pack->yaw             = 0.0f;
	uc_tx_pack->pitch           = 0.0f;
	uc_tx_pack->accel_x         = 0;
	uc_tx_pack->accel_y         = 0;
	uc_tx_pack->wheel_rpm[0]    = 0.0f;
	uc_tx_pack->wheel_rpm[1]    = 0.0f;
	uc_tx_pack->wheel_rpm[2]    = 0.0f;
	uc_tx_pack->wheel_rpm[3]    = 0.0f;
	uc_tx_pack->checksum        = -1;
}


void uc_board_response_pack_init(UC_Board_Response_Pack_t *uc_response_pack) {
	uc_response_pack->header          = UC_BOARD_RESPONSE_PACK_HEADER;
	uc_response_pack->salt            = -1;
	uc_response_pack->header_checksum = -1;
	uc_response_pack->response_code   = -1;
	uc_response_pack->checksum        = -1;
}


void uc_pc_command_pack_init(UC_PC_Command_Pack_t *uc_pc_command_pack) {
	uc_pc_command_pack->header          = UC_PC_COMMAND_PACK_HEADER;
	uc_pc_command_pack->salt            = -1;
	uc_pc_command_pack->header_checksum = -1;
	uc_pc_command_pack->command_code    = -1;
	uc_pc_command_pack->data[0]         = 0;
	uc_pc_command_pack->data[1]         = 0;
	uc_pc_command_pack->data[2]         = 0;
	uc_pc_command_pack->data[3]         = 0;
	uc_pc_command_pack->data[4]         = 0;
	uc_pc_command_pack->data[5]         = 0;
	uc_pc_command_pack->data[6]         = 0;
	uc_pc_command_pack->data[7]         = 0;
	uc_pc_command_pack->data[8]         = 0;
	uc_pc_command_pack->data[9]         = 0;
	uc_pc_command_pack->data[10]        = 0;
	uc_pc_command_pack->data[11]        = 0;
	uc_pc_command_pack->checksum        = -1;
}


uint32_t calculate_checksum(void *buffer_ptr, size_t buffer_size) {
	/*
	 * Calculates checksum of a continuous section of memory. This
	 * is often used for error detection in pack communication with
	 * the mini-PC.
	 *
	 * (BEWARE) This function assumes that pack_size is a multiple
	 * of 4.
	 *
	 * PARAMETERS:
	 * 	-
	 */
	const uint32_t MOD = 1000000007;
	uint32_t* data = (uint32_t*) buffer_ptr;
	uint32_t checksum = 0;

	// Calculate checksum only for data portion of a packet.
	// Assumes that the packet checksum is uint32_t and is the last
	// field of the packet struct.
	for (size_t i = 0; i < buffer_size / 4 - 1; i++){
		checksum = (checksum + (data[i] % MOD)) % MOD;
	}
	return checksum;
}


uint16_t uc_get_pack_size(uint8_t *pack_ptr) {
	switch (uc_get_header(pack_ptr)) {
	case UC_AUTO_AIM_PACK_HEADER: return UC_AUTO_AIM_PACK_SIZE;
	case UC_BOARD_RESPONSE_PACK_HEADER: return UC_BOARD_RESPONSE_PACK_SIZE;
	case UC_IMU_DATA_PACK_HEADER: return UC_IMU_DATA_PACK_SIZE;
	default:                      return 0;
	}
}


uint8_t uc_get_header(uint8_t *pack_ptr) {
	return uc_pack_buffer[0];
}


void uc_receive_pack() {
	/*
	 * Receives packs from mini-PC through UART with DMA.
	 *
	 * Data is first received in uc_input_buffer.
	 * If there are no errors in the received data, then the data
	 * is copied to uc_rx_pack to be used.
	 *
	 * Check uc_on_RxCplt() for error detection.
	 */
	if (uc_valid_header == 1) { // New header received.
		// Retrieve pack into buffer.
		HAL_UART_Receive_DMA(&huart1, uc_pack_buffer + UC_HEADER_SIZE, uc_get_pack_size(uc_pack_buffer) - UC_HEADER_SIZE);
	} else {
		HAL_UART_Receive_DMA(&huart1, uc_pack_buffer, UC_HEADER_SIZE);
		uc_valid_header = 0;
	}
}


uint8_t uc_send_pack(void* pack, uint16_t pack_size) {
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
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, pack, pack_size, 100);
	if (status == HAL_OK) {
		return 0;
	} else {
		return 1;
	}
}


void uc_on_RxCplt() {
	/*
	 * This is executed after pack is completely received.
	 * The following happens:
	 *  1. Compare the received checksum and checksum of uc_input_buffer.
	 *     If checksums match then copy data in uc_input_buffer to uc_rx_pack.
	 *  2. Send a UC_Response_Pack_t back to mini-PC.
	 *  3. Start reception of another pack.
	 */
	
	if (uc_valid_header == 1) {
		// Check if new pack is valid.
		uint32_t pack_checksum = calculate_checksum(uc_pack_buffer, uc_get_pack_size(uc_pack_buffer));
		uint32_t sent_checksum = *((uint32_t*) uc_pack_buffer + (uc_get_pack_size(uc_pack_buffer) / 4) - 1);
		if (pack_checksum == sent_checksum) {
			xQueueSendFromISR(UC_Pack_Queue, uc_pack_buffer, NULL);
		}
		uc_valid_header = 0;
	} else {
		// Check if new header is valid.
		uint16_t header_checksum = uc_pack_buffer[2] + 256 * uc_pack_buffer[3];
		if (uc_pack_buffer[0] + uc_pack_buffer[1] == header_checksum) {
			uc_valid_header = 1;
		}
	}
	uc_receive_pack();
}


#endif /*__AUTO_AIM_C__*/
