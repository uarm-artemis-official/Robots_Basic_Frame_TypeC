/*
******************************************************************************
* @file           : Referee_App.c
* @brief      	  : Referee system related files
* @created time	  : Jul, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/

#ifndef __REFEREE_APP_C__
#define __REFEREE_APP_C__

#include "main.h"
#include "string.h"
#include "Chassis_App.h"
#include <crc.h>
#include <Referee_App.h>
#include "referee_ui.h"

extern UART_HandleTypeDef huart1;
extern int16_t referee_parsed_flag;
extern uint8_t referee_timeout_counter;
extern uint8_t referee_timeout_check_flag;
/*
 *  @Referee System Note
 *		JUL, 2023: Use UART3 DMA IT to read the data from referee system intead of freertos task
 *
 * 	Helpful cmd index
 * 		0x0001  Competition status data				 3 Hz
 * 		0x0003  Robot HP data in competition		 3 Hz
 * 		0x0201  Robot status data					10 Hz
 * 		0x0202	Real-time power and heat data       50 Hz
 * 		0x0207  Real-time shoot/launching data		Real-Time(as launching)
 * 		0x0301  Interaction data between robots		10 Hz
 * 		0x0302  interface of the custom controller  30 Hz
 *
 * 		CMD id 0x0301 allows us to draw a customized graphic interface for operators
 *
 * */

uint8_t ref_tx_frame[MAX_REF_TX_DATA_LEN]; // Data pool

/**
  * @brief     main ref sys task function
  * @param[in] None
  * @retval    None
  */
void Referee_Task_Func(void const * argument){
	referee_init(&referee);

	uint8_t temp_ref_pack[MAX_REF_BUFFER_SZIE] = {0};

	/* Engage UART 1 */
	referee_dma_reset();

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(1); // task exec period 10ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	for(;;){
		while (uxQueueMessagesWaiting(Ref_Pack_Queue) > 0) {
			referee_timeout_check_flag = 0; // Disable the counter check
			xQueueReceive(Ref_Pack_Queue, temp_ref_pack, 0);

			/* Determime the robot color if first comm */
			if(referee.robot_color == UNKOWN){
				if(referee.robot_status_data.robot_id <= 11 && referee.robot_status_data.robot_id > 0)
					referee.robot_color = RED;
				else if(referee.robot_status_data.robot_id > 11)
					referee.robot_color = BLUE;
			}
			referee_read_data(&referee, temp_ref_pack);

			/* Reset referee rx buffer */
			memset(temp_ref_pack, 0, sizeof(temp_ref_pack));// Wipe the temp buffer

			/* Delay until wake time */
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}

		/* Check if there is congestion in DMA channel */
		if(referee_parsed_flag == 0){
			/* Set timeout check flag */
			referee_timeout_check_flag = 1;

			if(referee_timeout_counter >= 10) { // 1 second no recv data from server
				/* Reset DMA */
				referee_dma_reset();
				/* Reset timeoout counter */
				referee_timeout_counter = 0;
			}
		}

//		memcpy(temp_ref_pack, ref_rx_frame, sizeof(ref_rx_frame));


//		if(referee_parsed_flag){
//			/* Determime the robot color if first comm */
//			if(referee.robot_color == UNKOWN){
//				if(referee.robot_status_data.robot_id <= 11 && referee.robot_status_data.robot_id > 0)
//					referee.robot_color = RED;
//				else if(referee.robot_status_data.robot_id > 11)
//					referee.robot_color = BLUE;
//			}
//			referee_read_data(&referee, temp_ref_pack);
//		}
//
//		/* Reset referee rx buffer */
//		memset(temp_ref_pack, 0, sizeof(temp_ref_pack));// Wipe the temp buffer

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


/**
  * @brief     init ref sys struct
  * @param[in] main ref struct
  * @retval    None
  */
void referee_init(Referee_t *ref){

	memset(&(ref->header), 0, sizeof(frame_header_t));

	memset(&(ref->game_status_data),  0, sizeof(game_status_t));
	memset(&(ref->HP_data), 		  0, sizeof(game_robot_HP_t));
	memset(&(ref->robot_status_data), 0, sizeof(robot_status_t));
	memset(&(ref->power_heat_data),   0, sizeof(power_heat_data_t));
	memset(&(ref->shoot_data), 		  0, sizeof(shoot_data_t));
	memset(&(ref->ui_figure_struct_data),   0, sizeof(robot_interaction_data_t));
	memset(&(ref->custom_robot_data), 0, sizeof(custom_robot_data_t));

	ref->robot_status_data.robot_level = 1;
	ref->first_drawing_flag = 1;
	ref->cur_sending_count = 0;
	ref->ref_cmd_id = IDLE_ID;
	ref->ui_intrect_data.data_cmd_id = IDLE_ID;
	ref->robot_color = UNKOWN;
}

/**
  * @brief     init ref sys struct
  * @param[in] main ref struct
  * @param[in] received frame (array) from ref sys // abodoned, directly copy within uart3 dma
  * @retval    None
  */
void referee_read_data(Referee_t *ref, uint8_t *rx_frame){
	if (rx_frame == NULL) {
		// frame is NULL, return
		return;
	}
	/* copy frame header */
	memcpy(&ref->header, rx_frame, HEADER_LEN);

	/* frame header CRC8 verification */
	// FIXME: We don't know if we still need crc8 verification. if not , probably just update the pointer
	if(ref->header.sof == SOF_ID){
		if(Verify_CRC8_Check_Sum(&(ref->header), HEADER_LEN) == 1){

			/* Reset timeout flag */
			referee_timeout_check_flag = 0;
			referee_timeout_counter = 0;

			/* successfully verified */
			ref->ref_cmd_id = *(uint16_t *)(rx_frame + HEADER_LEN); //point to the addr of the cmd id (rx_frame[6] << 8 | rx_frame[5])

			memcpy(ref->ref_data, rx_frame + HEADER_LEN + CMD_LEN, sizeof(ref->ref_data));//pointer to the beginning of the data addr

			/* parse the frame and get referee data */
			switch(ref->ref_cmd_id){
				case GAME_STAT_ID: {
					memcpy(&(ref->game_status_data), ref->ref_data, sizeof(game_status_t));ref->ref_cmd_id = IDLE_ID;;break;
				}
				case GMAE_HP_ID: {
					memcpy(&(ref->HP_data), ref->ref_data, sizeof(game_robot_HP_t));ref->ref_cmd_id = IDLE_ID;break;
				}
				case ROBOT_STAT_ID: {
					memcpy(&(ref->robot_status_data), ref->ref_data, sizeof(robot_status_t));ref->ref_cmd_id = IDLE_ID;break;
				}
				case POWER_HEAT_ID: {
					memcpy(&(ref->power_heat_data), ref->ref_data, sizeof(power_heat_data_t));ref->ref_cmd_id = IDLE_ID;break;
				}
				case SHOOT_ID: {
					memcpy(&(ref->shoot_data), ref->ref_data, sizeof(shoot_data_t));ref->ref_cmd_id = IDLE_ID;break;
				}
				default: {
					break;
				}
			}

			if (*(rx_frame + HEADER_LEN + CMD_LEN + sizeof(ref->ref_data) + CRC_LEN) == 0xA5){ // Parsed multi-frame in one pack if needed
					referee_read_data(ref, rx_frame + HEADER_LEN + CMD_LEN + sizeof(ref->ref_data) + CRC_LEN);
				}
		}
	}
	else{
		ref->ref_cmd_id = IDLE_ID;

		/* Set timeout check flag */
		referee_timeout_check_flag = 1;

		if(referee_timeout_counter >= 10) { // 1 second no recv data from server
			/* Reset DMA */
			referee_dma_reset();
			/* Reset timeoout counter */
			referee_timeout_counter = 0;
		}

	}

	/* Reset referee parsed flag */
	referee_parsed_flag = 0;

	/* Resume UASRT DMA Recx */
	HAL_UART_Receive_DMA(&huart1, ref_rx_frame, sizeof(ref_rx_frame));

}

void referee_dma_reset(void){
	/* Reset DMA */
	HAL_UART_DMAStop(&huart1);

	/* Clear rx buffer */
	memset(ref_rx_frame, 0, MAX_REF_BUFFER_SZIE);

	/* Resume UASRT DMA Recx */
	HAL_UART_Receive_DMA(&huart1, ref_rx_frame, sizeof(ref_rx_frame));
}

/* Drawing the UI */
uint8_t pack_seq=0;
/**
  * @brief     Referee UI send
  * @param[in] cmd_id The id sent , usually the ui id 0x0301
  * @param[in] pdata the data frame to be sent
  * @param[in] len The length of the actual data frame
  * @retval    None
  */
void referee_send_ui_data(uint16_t cmd_id, uint8_t *p_data, uint16_t len){

	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;   // length of data frame

	memset(ref_tx_frame, 0, frame_length);  // Refresh the tx data buffer

	/* Packing the tx header */
	ref_tx_frame[0] = SOF_ID; 								// First byte of the data pool, 1 byte
	memcpy(&ref_tx_frame[1], (uint8_t*)&len, sizeof(len));  // Actual data len in the data pool, 2 bytes
	ref_tx_frame[3] = pack_seq;								// Pack sequence, start from 3, 1 byte
	Append_CRC8_Check_Sum(ref_tx_frame, HEADER_LEN);  		// CRC8 Calibration required by referee system

	/* Pack cmd id */
	memcpy(&ref_tx_frame[HEADER_LEN],(uint8_t*)&cmd_id, CMD_LEN);

	/* Pack actual data frame */
	memcpy(&ref_tx_frame[HEADER_LEN+CMD_LEN], p_data, len);
	Append_CRC16_Check_Sum(ref_tx_frame, frame_length);  // CRC16 Calibration

	if (pack_seq == 0xff)
		pack_seq = 0;
	else
		pack_seq++;

	HAL_UART_Transmit_DMA(&huart1, ref_tx_frame, frame_length);
}

void referee_set_ui_data(Referee_t *ref, referee_ui_t ui_type){

   if(ref->robot_status_data.robot_id == 3 || ref->robot_status_data.robot_id == 4 ||
		ref->robot_status_data.robot_id == 103 || ref->robot_status_data.robot_id == 104){ // Infantry
		ref->ui_intrect_data.sender_id=ref->robot_status_data.robot_id;
		switch(ref->robot_status_data.robot_id){
			case 3:   ref->ui_intrect_data.receiver_id=0x0103;break;// Red team #3 infantry client
			case 4:   ref->ui_intrect_data.receiver_id=0x0104;break;// Red team #4 infantry client
			case 103: ref->ui_intrect_data.receiver_id=0x0167;break;// Blue team #3 infantry client
			case 104: ref->ui_intrect_data.receiver_id=0x0168;break;// Blue team #4 infantry client
		}

		switch(ui_type){
			case UI_ROBOT_ACT_MODE: referee_general_draw_act_mode(ref);break;
			/* Hero Draw marks */
			case UI_INFANTRY_MARK: referee_infantry_draw_marks(ref);break;
			default: break;
		}
	}
	else if(ref->robot_status_data.robot_id == 1 || ref->robot_status_data.robot_id == 101){ // Hero
		/* Set up */
		ref->ui_intrect_data.sender_id = ref->robot_status_data.robot_id;
		/* Set up client ID */
		switch(ref->robot_status_data.robot_id){
			case 1:   ref->ui_intrect_data.receiver_id = 0x0101;break;// Red team #1 Hero client
			case 101: ref->ui_intrect_data.receiver_id = 0x0165;break;// Blue team #1 Hero client
		}

		switch(ui_type){
			case UI_ROBOT_ACT_MODE: referee_general_draw_act_mode(ref);break;
			/* Hero Draw marks */
			case UI_HERO_MARK: referee_hero_draw_marks(ref);break;
			default: break;

		}
	}

}



#endif
