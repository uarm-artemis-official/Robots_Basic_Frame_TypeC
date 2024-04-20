/*
 * UC_Comm_App.c
 *
 *  Created on: Mar. 18, 2024
 *      Author: James Fu
 */


#include <PC_UART_App.h>


void PC_UART_Func() {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // task exec period 1ms

	uint8_t new_pack_buffer[UC_PACK_BUFFER_SIZE];
	uc_auto_aim_pack_init(&uc_rx_pack);
	uc_imu_data_pack_init(&uc_tx_pack);
	uc_pc_command_pack_init(&uc_pc_command_pack);

	uc_valid_header = 0;
	uc_receive_pack();
	while (1) {
		xLastWakeTime = xTaskGetTickCount();

		while (uxQueueMessagesWaiting(UC_Pack_Queue) > 0) {
			xQueueReceive(UC_Pack_Queue, new_pack_buffer, 0);
			switch (uc_get_header(new_pack_buffer)) {
			case UC_AUTO_AIM_PACK_HEADER:
				memcpy(&uc_rx_pack, new_pack_buffer, UC_AUTO_AIM_PACK_SIZE);
				break;
			case UC_PC_COMMAND_PACK_HEADER:
				memcpy(&uc_pc_command_pack_init, new_pack_buffer, UC_PC_COMMAND_PACK_SIZE);
				break;
			default:
				break;
			}
		}

		// IMU Transmission
		// TODO: Send chassis x and y acceleration, robot color, and wheel RPMs.
		// This requires enabling IMU task for chassis and modifying Comm task.
		uc_tx_pack.pitch = imu.ahrs_sensor.pitch;
		uc_tx_pack.yaw = imu.ahrs_sensor.yaw;
		uc_send_pack(&uc_tx_pack, UC_IMU_DATA_PACK_SIZE);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
