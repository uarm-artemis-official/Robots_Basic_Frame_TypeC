///*
//******************************************************************************
//* @file           : Referee_App.h
//* @brief      	  : Referee system related files
//* @created time	  : Apr, 2024
//* @author         : Haoran
//*
//******************************************************************************
//* Copyright (c) 2023 UARM Artemis.
//* All rights reserved.
//******************************************************************************
//*/
//
//
//#ifndef __SRC_REFEREE_APP_H__
//#define __SRC_REFEREE_APP_H__
//
//
//#include "main.h"
//#include "stm32f4xx.h"
//#include "referee_ui.h"
//
///* define general declarations for gimbal task here */
//#define SOF_ID    	 0xA5  //fixed sof value
//#define HEADER_LEN   sizeof(frame_header_t) // 5
//#define CMD_LEN      2    //cmd_id bytes
//#define CRC_LEN      2    //crc16 bytes
//#define MAX_REF_BUFFER_SZIE 256
//#define MAX_REF_TX_DATA_LEN 128
//
///* extern global variables here */
//
//
///* define user created variables here */
//extern uint8_t ref_rx_frame[MAX_REF_BUFFER_SZIE];
//extern uint8_t ref_tx_frame[MAX_REF_TX_DATA_LEN];
//
//
///* functions declaration here */
//void Referee_Task_Func(void const * argument);
//void referee_init(Referee_t *ref);
//void referee_read_data(Referee_t *ref, uint8_t *rx_frame);
//void referee_set_ui_data(Referee_t *ref, referee_ui_t ui_type);
//void referee_send_ui_data(uint16_t cmd_id, uint8_t *p_data, uint16_t len);
//void referee_dma_reset(void);
//
//
//
//
//
//#endif /*__SRC_REFEREE_APP_C__*/
