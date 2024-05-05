/*
 * UC_Comm_App.h
 *
 *  Created on: Mar. 18, 2024
 *      Author: James Fu
 */

#ifndef SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_
#define SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_


#include "imu.h"
#include "auto_aim.h"
#include "cmsis_os.h"


extern QueueHandle_t UC_Pack_Queue;
extern IMU_t imu;

// GLOBALS
UC_Auto_Aim_Pack_t uc_auto_aim_pack;
UC_Board_Data_Pack_t uc_board_data_pack;
UC_Flow_Control_Pack_t uc_flow_control_pack;
uint8_t uc_pack_input_buffer[UC_PACK_SIZE];
extern uint8_t aa_pack_recv_flag;


void uc_on_RxCplt();
void PC_UART_Func();

#endif /* SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_ */
