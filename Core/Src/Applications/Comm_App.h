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

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

/*
 * @brief Communication app main handler
 * @note None
 * */

// typedef enum {
// 	UART_COMM_MODE = 0,
// 	CAN_COMM_MODE,
// }CommMode_t;

// typedef struct{
// 	int16_t Not_implement_yet;
// }UartComm_t;

// typedef struct{
// 	uint32_t comm_id;    //refer to sender
// 	uint32_t comm_rx_id; //refer to receiver
// 	int16_t tx_data[4];
// 	int16_t rx_data[TOTAL_COMM_ID][4];

// 	void (*can_send_comm_data)(CAN_HandleTypeDef* hcan, int16_t* send_data, uint32_t tx_id);
// 	void (*can_recv_comm_data)(CAN_HandleTypeDef* hcan, uint32_t data_len, int16_t (*rx_buffer)[TOTAL_COMM_ID][4]);
// }CanComm_t;

/* main comm app struct */
//typedef struct{
//	CommMode_t comm_mode;
//	UartComm_t uart_comm;
//	CanComm_t can_comm;
//
//	/* message subscription list*/
//	CommMessageSublist_t sub_list;
//}BoardComm_t;

class CommApp : public RTOSApp<CommApp> {
   private:
    IMessageCenter& message_center;
    IDebug& debug;
    ICanComm& can_comm;
    BoardStatus_t board_status;

   public:
    static constexpr const uint32_t LOOP_PERIOD_MS = COMM_TASK_EXEC_TIME;

    CommApp(IMessageCenter& message_center, IDebug& debug, ICanComm& can_comm);
    void loop();
    void init();
};

#endif /*__COMM_APP_H__*/
