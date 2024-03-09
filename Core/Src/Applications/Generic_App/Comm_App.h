/*******************************************************************************
* @file           : Comm_App.h
* @brief          : communication real time task between boards
* @created time	  : Jul, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __COMM_APP_H__
#define __COMM_APP_H__

#include <auto_aim_pack.h>
#include <Gimbal_App.h>
#include <queue_m.h>
#include <comms.h>

#define USE_UART_DMA 1

/* Define used can id */
//higher id, lower priority
//FIXME: adjust the priority
#define IDLE_COMM_ID    0x300
#define ANGLE_COMM_ID   0x301
#define RC_COMM_ID      0x302
#define PC_COMM_ID      0x303
#define KEY_COMM_ID     0x304
#define REF_COMM_ID   	0x305
#define ADD_KEY_ID 		0x306
#define TOTAL_COMM_ID   7

#define ANGLE_IDX    (ANGLE_COMM_ID-IDLE_COMM_ID)
#define RC_IDX       (RC_COMM_ID-IDLE_COMM_ID)
#define PC_IDX    	 (PC_COMM_ID-IDLE_COMM_ID)
#define REF_IDX      (REF_COMM_ID-IDLE_COMM_ID)
#define KEY_IDX   	 (KEY_COMM_ID-IDLE_COMM_ID)
#define ADD_KEY_IDX  (ADD_KEY_ID-IDLE_COMM_ID)


#define ANGLE_COMM_SCALE_FACTOR (32767.0f / PI - 500.0f) //32767 is the maximum size of int16_t														 // exp: since we need to transmit the float angle(-pi, pi), we need to transfer it														 // to int16_t, and rescale it in the receiver side.

/*
 * @brief Communication app main handler
 * @note None
 * */


typedef enum {
	UART_COMM_MODE = 0,
	CAN_COMM_MODE,
}CommMode_t;

typedef struct{
	int16_t Not_implement_yet;
}UartComm_t;

typedef struct{
	uint32_t comm_id;    //refer to sender
	uint32_t comm_rx_id; //refer to receiver
	int16_t tx_data[4];
	int16_t rx_data[TOTAL_COMM_ID][4];

	void (*can_send_comm_data)(CAN_HandleTypeDef* hcan, int16_t* send_data, uint32_t tx_id);
	void (*can_recv_comm_data)(CAN_HandleTypeDef* hcan, uint32_t data_len, int16_t (*rx_buffer)[TOTAL_COMM_ID][4]);
}CanComm_t;

/* main comm app struct */
typedef struct{
	CommMode_t comm_mode;
	UartComm_t uart_comm;
	CanComm_t can_comm;

	/* message subscription list*/
	CommMessageSublist_t sub_list;
}BoardComm_t;

BoardComm_t chassis_comm;
BoardComm_t gimbal_comm;
CanMessage_t canQueue[QUEUE_SIZE];
QueueManage_t canqm;

/* User Defined Varibales*/

/* extern global variables*/
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart6;
extern BoardStatusType board_status;
extern char pdata[PACKLEN];
extern float can_rx_scale_buffer[TOTAL_COMM_ID][4];
extern float can_tx_scale_buffer[TOTAL_COMM_ID][4];

/* declare function here */
void Comm_Task_Func(void const * argument);
void usart_comm_process(void);
void can_comm_process(BoardComm_t *comm);
void can_comm_subscribe_process(void);
void can_comm_reset_config(BoardComm_t *comm);
void can_send_comm_data(CAN_HandleTypeDef* hcan, int16_t* send_data, uint32_t tx_id);
void can_recv_comm_data(CAN_HandleTypeDef* hcan, uint32_t data_len, int16_t (*rx_buffer)[TOTAL_COMM_ID][4]);
void process_rx_data_i16tof(BoardComm_t *comm, float *output_buffer, float scale_factor, uint32_t idx);
void process_tx_data_ftoi16(float* input_data, int16_t* output_data, int length, float scale_factor);

#endif /*__COMM_APP_H__*/
