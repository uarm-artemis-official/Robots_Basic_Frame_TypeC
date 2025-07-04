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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

#include "usart.h"

// UC COMMUNICATION FUNCTIONS
uint8_t uc_send_bytes(uint8_t* bytes, size_t size);

void uc_start_receive(uint8_t* pack_buffer, size_t buffer_size);
void uc_restart_receive(uint8_t* pack_buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /*__AUTO_AIM_H__*/
