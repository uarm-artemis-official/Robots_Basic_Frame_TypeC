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

#include "public_defines.h"
#include "usart.h"

// PACK DEFINITIONS
#define MAX_PACK_BUFFER_SIZE 64  // Measured in bytes.
#define PACK_HEADER_SIZE 4
#define PACK_TRAILER_SIZE 4

// PACK HEADERS
#define UC_AUTO_AIM_HEADER 0x01
#define UC_BOARD_DATA_HEADER 0x02
#define UC_FLOW_CONTROL_HEADER 0x04

// PACK STRUCTS
// upper computer -> MCU
typedef struct {
    uint8_t target_num;
    float delta_yaw;
    float delta_pitch;
} UC_Auto_Aim_Pack_t;

typedef struct {
    uint8_t control_code;
    uint32_t sequence_number;
    uint32_t acknowledge;
} UC_Flow_Control_Pack_t;

// MCU -> upper computer
typedef struct {
    uint8_t robot_color;  // 0: red, 1: blue

    // imu data
    float pitch;
    float yaw;
    float accel_x;
    float accel_y;

    float wheel_rpm[4];  // from chassis wheels
} UC_Board_Data_Pack_t;

// OTHER STRUCTS
typedef union pack_checksum {
    uint32_t as_integer;
    uint8_t as_bytes[PACK_TRAILER_SIZE];
} UC_Checksum_t;

typedef struct {
    uint8_t header_id;
    uint8_t header_size;
    uint8_t data_size;
    uint8_t trailer_size;
} Pack_Metadata_t;

// INIT FUNCTIONS
void pack_init(void* pack_struct, uint8_t data_size);

UC_Checksum_t calculate_checksum(void* data, size_t size);

/*
 * Checks if the header in uc_pack_input_buffer is recognized header.
 * All recognized headers are defined under PACK HEADERS macros above.
 *
 * RETURNS:
 *  1 - if the header is a recognized header.
 *  0 - otherwise.
 */
uint8_t is_valid_header(uint8_t* input_buffer);

// UC COMMUNICATION FUNCTIONS
uint8_t uc_send_board_data(UC_Board_Data_Pack_t* pack);
uint8_t uc_send(void* pack);
uint8_t uc_send_bytes(uint8_t* bytes, size_t size);

void start_receive(uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE]);
void restart_receive(uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE]);

uint8_t get_data_size(uint8_t header_id);

#ifdef __cplusplus
}
#endif

#endif /*__AUTO_AIM_H__*/
