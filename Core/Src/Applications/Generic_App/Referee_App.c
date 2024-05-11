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

#ifndef __SRC_REFEREE_APP_C__
#define __SRC_REFEREE_APP_C__

#include "main.h"
#include "string.h"
#include "usart.h"
#include "Chassis_App.h"
#include <crc.h>
#include <Referee_App.h>

extern UART_HandleTypeDef huart1;
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

int16_t referee_parsed_flag = 0;
Referee_t referee;

/**
  * @brief     main ref sys task function
  * @param[in] None
  * @retval    None
  */
void Referee_Task_Func(void const * argument){
	referee_init(&referee);

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(10); // task exec period 10ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		referee_read_data(&referee, ref_rx_frame);

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
	memset(&(ref->ui_intrect_data),   0, sizeof(robot_interaction_data_t));
	memset(&(ref->custom_robot_data), 0, sizeof(custom_robot_data_t));

	memset(&(ref->ui_figure_data), 0, sizeof(interaction_figure_t));
	memset(&(ref->ui_del_fig_data), 0, sizeof(interaction_layer_delete_t));
	memset(&(ref->ui_figure_struct_data), 0, sizeof(interaction_figure_4_t));
	memset(&(ref->ui_custom_data), 0, sizeof(ext_client_custom_character_t));
	memset(&(ref->custom_robot_data), 0, sizeof(custom_robot_data_t));

	ref->robot_status_data.robot_level = 1;
	ref->ref_cmd_id = IDLE_ID;
}

/**
  * @brief     init ref sys struct
  * @param[in] main ref struct
  * @param[in] received frame (array) from ref sys // abodoned, directly copy within uart3 dma
  * @retval    None
  */
void referee_read_data(Referee_t *ref, uint8_t *rx_frame){

	referee_parsed_flag = 0;

	if (rx_frame == NULL) {
		// frame is NULL, return
		return;
	}
	/* copy frame header */
	memcpy(&ref->header, rx_frame, HEADER_LEN);

	/* frame header CRC8 verification */
	// FIXME: We don't know if we still need crc8 verification. if not , probably just update the pointer
//	if(ref->header.sof == SOF_ID && Verify_CRC8_Check_Sum(&(ref->header), HEADER_LEN) == 1){
	if(ref->header.sof == SOF_ID){
		/* successfully verified */
		ref->ref_cmd_id = *(uint16_t *)(rx_frame + HEADER_LEN); //point to the addr of the cmd id
	}
	else{
		ref->ref_cmd_id = IDLE_ID;
	}

	//uint8_t ref_data_index = HEADER_LEN + CMD_LEN;// an index value pointed to current data addr
	memcpy(ref->ref_data, rx_frame + HEADER_LEN + CMD_LEN, sizeof(ref->ref_data));//pointer to the beginning of the data addr


	/* parse the frame and get referee data */
	switch(ref->ref_cmd_id){
		case GAME_STAT_ID: {
			memcpy(&(ref->game_status_data), ref->ref_data, sizeof(game_status_t));break;
		}
		case GMAE_HP_ID: {
			memcpy(&(ref->HP_data), ref->ref_data, sizeof(game_robot_HP_t));break;
		}
		case ROBOT_STAT_ID: {
			memcpy(&(ref->robot_status_data), ref->ref_data, sizeof(robot_status_t));break;
		}
		case POWER_HEAT_ID: {
			memcpy(&(ref->power_heat_data), ref->ref_data, sizeof(power_heat_data_t));break;
		}
		case SHOOT_ID: {
			memcpy(&(ref->shoot_data), ref->ref_data, sizeof(shoot_data_t));break;
		}

	}

	referee_parsed_flag = 1;
}


/* Drawing the UI */
uint8_t pack_seq=0;
void referee_pack_ui_data(uint8_t sof,uint16_t cmd_id, uint8_t *p_data, uint16_t len){
	uint8_t  tx_buff[MAX_REF_TX_DATA_LEN]; // Data pool
	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;   // length of data frame

	memset(tx_buff,0, frame_length);  // Refresh the data pool

	/* Packing the handler */
	tx_buff[0] = SOF_ID; // First byte of the data pool
	memcpy(&tx_buff[1],(uint8_t*)&len, sizeof(len));// Actual data len in the data pool
	tx_buff[3] = pack_seq;// Pack sequence
	Append_CRC8_Check_Sum(tx_buff, HEADER_LEN);  // CRC8 Calibration required by referee system

	/* Pack cmd id */
	memcpy(&tx_buff[HEADER_LEN],(uint8_t*)&cmd_id, CMD_LEN);

	/* Pack final data */
	memcpy(&tx_buff[HEADER_LEN+CMD_LEN], p_data, len);
	Append_CRC16_Check_Sum(tx_buff,frame_length);  // CRC16 Calibration

	if (pack_seq == 0xff)
		pack_seq=0;
	else
		pack_seq++;

	/* Transmit the data */
	HAL_UART_Transmit_DMA(&huart1, tx_buff, frame_length);
}

void referee_set_ui_data(Referee_t *ref){
	    ref->ui_intrect_data.data_cmd_id=0x0104;// Draw 7 blocks
	    //FIXME: check the color and robot first here
	    if(ref->robot_status_data.robot_id == 3 || ref->robot_status_data.robot_id == 4 || \
	       ref->robot_status_data.robot_id == 103 || ref->robot_status_data.robot_id == 104){ // Infantry
			ref->ui_intrect_data.sender_ID=ref->robot_status_data.robot_id;
			switch(ref->robot_status_data.robot_id){
				case 3: ref->ui_intrect_data.receiver_ID=0x0103;break;// Red team #3 infantry client
				case 4: ref->ui_intrect_data.receiver_ID=0x0104;break;// Red team #4 infantry client
				case 103: ref->ui_intrect_data.receiver_ID=0x0167;break;// Blue team #3 infantry client
				case 104: ref->ui_intrect_data.receiver_ID=0x0168;break;// Blue team #4 infantry client
			}
			// The top three bytes represent the graphic name, used for graphic indexing,
			// which can be defined by yourself
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;// Graphic name

			//Graphic operations [0], 0: null operation; 1: add; 2: modify; 3: delete; 4: rename;
			//5: delete; 6: rename; 7: rename; 8: rename; 9: rename; 10: rename; 11: rename; 12: rename; 13: rename.
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].operate_tpye=1;
			ref->ui_intrect_data.graphic_custom_struct[0].graphic_tpye=0;//graphic type, 0 is straight line, check user manual for others
			ref->ui_intrect_data.graphic_custom.graphic_data_struct[0].layer=1;//number of layers
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].color=1;//color
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_angle=0;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_angle=0;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].width=1;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH/2;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_y=SCREEN_WIDTH/2-300;
			ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].radius=0;

	    }
	    else if(ref->robot_status_data.robot_id == 1 || ref->robot_status_data.robot_id == 101){ // Hero
	 			ref->ui_intrect_data.sender_ID=ref->robot_status_data.robot_id;
	 			switch(ref->robot_status_data.robot_id){
	 				case 1: ref->ui_intrect_data.receiver_ID=0x0101;break;// Red team #3 Hero client
	 				case 101: ref->ui_intrect_data.receiver_ID=0x0165;break;// Blue team #3 Hero client
	 			}
	 			// The top three bytes represent the graphic name, used for graphic indexing,
				// which can be defined by yourself
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[0] = 97;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[1] = 97;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].graphic_name[2] = 0;// Graphic name

				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].operate_tpye=1;
				ref->ui_intrect_data.graphic_custom_struct[0].graphic_tpye=0;//graphic type, 0 is straight line, check user manual for others
				ref->ui_intrect_data.graphic_custom.graphic_data_struct[0].layer=1;//number of layers
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].color=1;//color
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_angle=0;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_angle=0;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].width=1;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_x=SCREEN_LENGTH/2;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].start_y=SCREEN_WIDTH/2;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_x=SCREEN_LENGTH/2;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].end_y=SCREEN_WIDTH/2-300;
				ref->ui_intrect_data.graphic_custom.grapic_data_struct[0].radius=0;
	 	    }


}


#endif /*__DIRECTORY_ANY_CODE_C__*/
