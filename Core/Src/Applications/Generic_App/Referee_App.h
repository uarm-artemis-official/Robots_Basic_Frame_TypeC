/*
******************************************************************************
* @file           : Referee_App.h
* @brief      	  : Referee system related files
* @created time	  : Apr, 2024
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/


#ifndef __SRC_REFEREE_APP_H__
#define __SRC_REFEREE_APP_H__


#include "main.h"
#include "stm32f4xx.h"


/* define general declarations for gimbal task here */
#define SOF_ID    	 0xA5  //fixed sof value
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmd_id bytes
#define CRC_LEN      2    //crc16 bytes
#define MAX_REF_RX_DATA_LEN 41 //0x020B=40 + 1 as NUll buffer(maybe?)
#define MAX_REF_TX_DATA_LEN 128

/* define user structure here */
/* copy those from DJI referee manual */
typedef struct __attribute__((__packed__))//0x0001
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
}game_status_t;

typedef struct __attribute__((__packed__))//0x0003
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
}game_robot_HP_t;

typedef struct __attribute__((__packed__))//0x0201
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
}robot_status_t;

typedef struct __attribute__((__packed__))//0x0202
{
	uint16_t chassis_voltage;
	uint16_t chassis_current;
	float chassis_power;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
}power_heat_data_t;

typedef struct __attribute__((__packed__))//0x0207
{
uint8_t bullet_type;
uint8_t shooter_number;
uint8_t launching_frequency;
float initial_speed;
}shoot_data_t;

typedef struct __attribute__((__packed__))//0x0301
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[113];//max length 113
}robot_interaction_data_t;

/* Sub struct for cmd id 0x0301 */
// to be implement

typedef struct __attribute__((__packed__))//0x0302
{
	uint8_t data[30]; // max 30
}custom_robot_data_t;

/* used id define */
typedef enum{
	IDLE_ID 		    = 0x0000,
	GAME_STAT_ID 		= 0x0001,
	GMAE_HP_ID   		= 0x0003,
	ROBOT_STAT_ID 		= 0x0201,
	POWER_HEAT_ID 		= 0x0202,
	SHOOT_ID	  		= 0x0207,
	INTERA_UI_ID  		= 0x0301,
	INTERRA_USER_DATA 	= 0x0302
}referee_id_t;

/* main referee system struct */
typedef struct __attribute__((__packed__))
{
  uint8_t  sof; //0xA5
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
}frame_header_t;

typedef struct __attribute__((__packed__))
{
	frame_header_t header;
	uint8_t ref_data[MAX_REF_RX_DATA_LEN];

	// rx data
	game_status_t 	  game_status_data;
	game_robot_HP_t   HP_data;
	robot_status_t 	  robot_status_data;
	power_heat_data_t power_heat_data;
	shoot_data_t      shoot_data;

	// tx data
	robot_interaction_data_t ui_intrect_data;
	custom_robot_data_t custom_robot_data;
	uint16_t ref_cmd_id;

}Referee_t;

extern Referee_t referee;


/* extern global variables here */
extern int16_t referee_parsed_flag;


/* define user created variables here */
extern uint8_t ref_rx_frame[256];


/* functions declaration here */
void referee_init(Referee_t *ref);
void referee_read_data(Referee_t *ref, uint8_t *rx_frame);
void Referee_Task_Func(void const * argument);



#endif /*__SRC_REFEREE_APP_C__*/
