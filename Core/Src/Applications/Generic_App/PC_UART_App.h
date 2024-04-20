/*******************************************************************************
* @file           : PC_UART_App.h
* @brief          : PC Communication with mcu usig UART interface
* @created time	  : Mar, 2024
* @author         : James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __PC_UART_APP_H__
#define __PC_UART_APP_H__

#include "auto_aim.h"
#include "imu.h"
#include "cmsis_os.h"
#include "Gimbal_App.h"


extern QueueHandle_t UC_Pack_Queue;
extern UART_HandleTypeDef huart1;
extern IMU_t imu;

// GLOBALS
UC_Auto_Aim_Pack_t uc_rx_pack;
UC_IMU_Data_Pack_t uc_tx_pack;
UC_PC_Command_Pack_t uc_pc_command_pack;


void PC_UART_Func();

#endif /* SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_ */
