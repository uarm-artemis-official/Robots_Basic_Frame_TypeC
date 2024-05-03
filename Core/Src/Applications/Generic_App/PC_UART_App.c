/*
 * UC_Comm_App.c
 *
 *  Created on: Mar. 18, 2024
 *      Author: James Fu
 */


#ifndef SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_C_
#define SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_C_

#include <PC_UART_App.h>

uint8_t aa_pack_recv_flag = 0;;


void start_receive() {
	uc_receive(uc_pack_input_buffer, 1);
}


void process_flow_control() {
	// TODO: Implement.
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

	// Pack checksum can never be exactly 0, so if any bits
	// in the checksum are set then a pack was just received.
	uint8_t checksum_bits = uc_pack_input_buffer[UC_PACK_SIZE - 1] |
			uc_pack_input_buffer[UC_PACK_SIZE - 2] |
			uc_pack_input_buffer[UC_PACK_SIZE - 3] |
			uc_pack_input_buffer[UC_PACK_SIZE - 4];

	if (checksum_bits != 0) {
		// Error-detection with checksum.
		UC_Checksum_t pack_checksum = calculate_checksum(uc_pack_input_buffer, UC_PACK_SIZE);
		uint32_t sent_checksum = *((uint32_t*) uc_pack_input_buffer + (UC_PACK_SIZE / 4) - 1);
		if (pack_checksum.as_integer == sent_checksum) {
			// If valid pack -> Send to PC_UART_App for further processing with UC_Pack_Queue.
			xQueueSendFromISR(UC_Pack_Queue, uc_pack_input_buffer, NULL);
		}
		// Clear input buffer.
		memset(uc_pack_input_buffer, 0, UC_PACK_SIZE);

		// Receive another pack header.
		uc_receive(uc_pack_input_buffer, 1);
	} else {
		if (is_valid_header(uc_pack_input_buffer)) {
			// Receive rest of pack (data + trailer) from UC.
			uc_receive(uc_pack_input_buffer + 1, UC_PACK_SIZE - 1);
		} else {
			// (Invalid header) Receive another header.
			uc_receive(uc_pack_input_buffer, 1);
		}
	}
}


void PC_UART_Func() {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // task exec period 1ms

	uint8_t new_pack_buffer[UC_PACK_SIZE];
	uint8_t idle_count = 0;

	uc_auto_aim_pack_init(&uc_auto_aim_pack);
	uc_board_data_pack_init(&uc_board_data_pack);
	uc_flow_control_pack_init(&uc_flow_control_pack);

	start_receive();
	while (1) {
		xLastWakeTime = xTaskGetTickCount();

		// Reset DMA receive every 0.5 seconds after receiving last pack.
		if (idle_count >= 100) {
			uc_stop_receive();
			start_receive();
			idle_count = 0;
		}

		while (uxQueueMessagesWaiting(UC_Pack_Queue) > 0) {
			xQueueReceive(UC_Pack_Queue, new_pack_buffer, 0);
			idle_count = 0;
			switch (new_pack_buffer[0]) {
			case UC_AUTO_AIM_HEADER:
				memcpy(&uc_auto_aim_pack, new_pack_buffer + UC_PACK_HEADER_SIZE, UC_AUTO_AIM_DATA_SIZE);
				aa_pack_recv_flag = 1;
				break;
			case UC_FLOW_CONTROL_HEADER:
				memcpy(&uc_flow_control_pack, new_pack_buffer, UC_FLOW_CONTROL_DATA_SIZE);
				process_flow_control();
				break;
			default:
				break;
			}
		}

		idle_count++;

		// IMU Transmission
		// TODO: Send chassis x and y acceleration, robot color, and wheel RPMs.
		// This requires enabling IMU task for chassis and modifying Comm task.
//		uc_board_data_pack.pitch = imu.ahrs_sensor;
//		uc_board_data_pack.yaw = imu.ahrs_sensor;
		uc_board_data_pack.robot_color = 2;
		uc_board_data_pack.pitch = 1.111f;
		uc_board_data_pack.yaw = 2.222f;
		uc_board_data_pack.accel_x = 0.999f;
		uc_board_data_pack.accel_y = 0.888f;
		uc_board_data_pack.wheel_rpm[0] = 0.1f;
		uc_board_data_pack.wheel_rpm[1] = 0.2f;
		uc_board_data_pack.wheel_rpm[2] = 0.3f;
		uc_board_data_pack.wheel_rpm[3] = 0.4f;
		uc_send_board_data(&uc_board_data_pack);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

#endif
