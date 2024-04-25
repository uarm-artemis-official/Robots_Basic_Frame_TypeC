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
#include "Chassis_App.h"
#include <crc.h>
#include <Referee_App.h>
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





#endif /*__DIRECTORY_ANY_CODE_C__*/
