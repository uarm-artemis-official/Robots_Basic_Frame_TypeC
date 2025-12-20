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

void uc_start_receive(uint8_t* pack_buffer, size_t buffer_size) {
    HAL_UART_Receive_DMA(&huart1, pack_buffer, buffer_size);
}

void uc_restart_receive(uint8_t* pack_buffer, size_t buffer_size) {
    HAL_UART_AbortReceive(&huart1);
    uc_start_receive(pack_buffer, buffer_size);
}

uint8_t uc_send_bytes(uint8_t* bytes, size_t size) {
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, bytes, size, 100);
    if (status == HAL_OK) {
        return 0;
    } else {
        return 1;
    }
}

#endif /*__AUTO_AIM_C__*/
