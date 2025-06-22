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

#include "pack_handler.h"

static Pack_Metadata_t pack_metadata[] = {
    {
        .header_id = UC_AUTO_AIM_HEADER,
        .data_size = sizeof(UC_Auto_Aim_Pack_t),
    },
    {
        .header_id = UC_BOARD_DATA_HEADER,
        .data_size = sizeof(UC_Board_Data_Pack_t),
    },
    {
        .header_id = UC_FLOW_CONTROL_HEADER,
        .data_size = sizeof(UC_Flow_Control_Pack_t),
    },
};

void uc_start_receive(uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE]) {
    HAL_UART_Receive_DMA(&huart1, pack_buffer, MAX_PACK_BUFFER_SIZE);
}

void uc_restart_receive(uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE]) {
    HAL_UART_AbortReceive(&huart1);
    uc_start_receive(pack_buffer);
}

UC_Checksum_t calculate_checksum(void* buffer_ptr, size_t buffer_size) {
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

    for (size_t i = 0; i < buffer_size / 4 - 1; i++) {
        running_checksum = (running_checksum + (data[i] % MOD)) % MOD;
    }

    UC_Checksum_t checksum_union;
    checksum_union.as_integer = running_checksum;
    return checksum_union;
}

uint8_t is_valid_header(uint8_t* input_buffer) {
    // All valid packs will have the header as the first byte in the input buffer.
    // Check under PACK HEADER in auto_aim.h to see macros defining recognized headers.
    for (size_t i = 0; i < sizeof(pack_metadata) / sizeof(Pack_Metadata_t);
         i++) {
        if (input_buffer[0] == pack_metadata[i].header_id) {
            return 1;
        }
    }
    return 0;
}

uint8_t uc_send_board_data(UC_Board_Data_Pack_t* board_data_pack) {
    uint8_t data_size = uc_get_data_size((uint8_t) UC_BOARD_DATA_HEADER);
    uint8_t pack_bytes[MAX_PACK_BUFFER_SIZE];
    pack_bytes[0] = UC_BOARD_DATA_HEADER;

    memcpy(pack_bytes + PACK_HEADER_SIZE, board_data_pack, data_size);

    UC_Checksum_t checksum =
        calculate_checksum(pack_bytes, MAX_PACK_BUFFER_SIZE);

    memcpy(pack_bytes + MAX_PACK_BUFFER_SIZE - PACK_TRAILER_SIZE,
           checksum.as_bytes, PACK_TRAILER_SIZE);
    uint8_t response = uc_send(pack_bytes);
    return response;
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
    return uc_send_bytes(pack, MAX_PACK_BUFFER_SIZE);
}

uint8_t uc_send_bytes(uint8_t* bytes, size_t size) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, bytes, size, 100);
    if (status == HAL_OK) {
        return 0;
    } else {
        return 1;
    }
}

uint8_t uc_get_data_size(uint8_t header_id) {
    for (size_t i = 0; i < sizeof(pack_metadata) / sizeof(Pack_Metadata_t);
         i++) {
        if (header_id == pack_metadata[i].header_id) {
            return pack_metadata[i].data_size;
        }
    }
    return -1;
}

#endif /*__AUTO_AIM_C__*/
