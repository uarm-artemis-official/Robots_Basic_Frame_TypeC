/*
 * UC_Comm_App.h
 *
 *  Created on: Mar. 18, 2024
 *      Author: James Fu
 */

#ifndef SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_
#define SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_

#include "auto_aim.h"
#include "imu.h"
#include "cmsis_os.h"


extern QueueHandle_t UC_Pack_Queue;
extern UART_HandleTypeDef huart1;
extern IMU_t imu;

// GLOBALS
UC_Auto_Aim_Pack_t uc_rx_pack;
UC_IMU_Data_Pack_t uc_tx_pack;
UC_PC_Command_Pack_t uc_pc_command_pack;


void PC_UART_Func();

#endif /* SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_ */
