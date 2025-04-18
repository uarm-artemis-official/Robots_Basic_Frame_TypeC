/*******************************************************************************
* @file           : PC_UART_App.c
* @brief          : Upper computer communication task
* @restructed     : Mar, 2024
* @maintainer     : James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __PC_UART_APP_C__
#define __PC_UART_APP_C__

#include "PC_UART_App.h"


static uint8_t new_pack_buffer[MAX_PACK_BUFFER_SIZE];


void process_flow_control() {
	// TODO: Implement.
}


BaseType_t uc_check_pack_integrity(uint8_t *pack_bytes, uint8_t pack_size) {
	// Pack checksum can never be exactly 0, so if any bits
	// in the checksum are set then a pack was just received.
	uint8_t checksum_bits = pack_bytes[pack_size - 1] |
			pack_bytes[pack_size - 2] |
			pack_bytes[pack_size - 3] |
			pack_bytes[pack_size - 4];
	UC_Checksum_t pack_checksum = calculate_checksum(pack_bytes, MAX_PACK_BUFFER_SIZE);
	uint32_t sent_checksum = *((uint32_t*) pack_bytes + (MAX_PACK_BUFFER_SIZE / 4) - 1);

	if (checksum_bits != 0 && pack_checksum.as_integer == sent_checksum && is_valid_header(pack_bytes)) {
		return pdTRUE;
	} else {
		return pdFALSE;
	}
}


static uint32_t idle_count = 0;
void PC_UART_Func() {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(PC_UART_TASK_EXEC_TIME);

	xLastWakeTime = xTaskGetTickCount();

	UC_Board_Data_Pack_t send_pack;
	pack_init(&send_pack, get_data_size(UC_BOARD_DATA_HEADER));
	send_pack.robot_color = 2;
	send_pack.pitch = 1.111f;
	send_pack.yaw = 2.222f;
	send_pack.accel_x = 0.999f;
	send_pack.accel_y = 0.888f;
	send_pack.wheel_rpm[0] = 0.1f;
	send_pack.wheel_rpm[1] = 0.2f;
	send_pack.wheel_rpm[2] = 0.3f;
	send_pack.wheel_rpm[3] = 0.4f;
	uc_send_board_data(&send_pack);

	start_receive();
	for (;;) {
		if (idle_count == 40) {
			restart_receive();
			idle_count = 0;
		}

		while (get_message(UC_PACK_IN, new_pack_buffer, 0) == pdTRUE) {
			if (uc_check_pack_integrity(new_pack_buffer, MAX_PACK_BUFFER_SIZE) == pdTRUE) {
				switch (new_pack_buffer[0]) {
					case UC_AUTO_AIM_HEADER:{
						UC_Auto_Aim_Pack_t aim_pack;
						memcpy(&aim_pack, new_pack_buffer + PACK_HEADER_SIZE, get_data_size(new_pack_buffer[0]));
						if (aim_pack.target_num > 0) {
							float deltas[] = { aim_pack.delta_yaw, aim_pack.delta_pitch };
							pub_message(AUTO_AIM, deltas);
						}
						break;
					}
					case UC_FLOW_CONTROL_HEADER:{
//						memcpy(&uc_flow_control_pack, new_pack_buffer, UC_FLOW_CONTROL_DATA_SIZE);
//						process_flow_control();
						break;
					}
					default:
						break;
				}
			}
			idle_count = 0;
		}
		idle_count = (idle_count + 1) % 100000;

		// IMU Transmission
		// TODO: Send chassis x and y acceleration, robot color, and wheel RPMs.
		// This requires enabling IMU task for chassis and modifying Comm task.
		send_pack.pitch += 0.1;
		uc_send_board_data(&send_pack);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

#endif
