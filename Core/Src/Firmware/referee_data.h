/*******************************************************************************
* @file           : referee_data.h
* @brief          : The function to draw referee ui
* @created time	  : May, 2024
* @author         : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __REFEREE_DATA_H__
#define __REFEREE_DATA_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void referee_transmit_data(uint8_t* data, uint16_t len);

// #define MAX_REF_RX_DATA_LEN 41  //0x020B=40 + 1 as NUll buffer(maybe?)
// typedef struct __attribute__((__packed__)) {
//     frame_header_t header;
// uint8_t ref_data[MAX_REF_RX_DATA_LEN];

//     // rx data
//     game_status_t game_status_data;
//     game_robot_HP_t HP_data;
//     robot_status_t robot_status_data;
//     power_heat_data_t power_heat_data;
//     shoot_data_t shoot_data;

//     // tx data
//     robot_interaction_data_t ui_intrect_data;
//     interaction_layer_delete_t ui_del_fig_data;
//     interaction_figure_t ui_figure_data;
//     interaction_figure_2_t ui_figure_draw_2_data;
//     interaction_figure_5_t ui_figure_draw_5_data;
//     interaction_figure_7_t ui_figure_struct_data;
//     ext_client_custom_character_t ui_custom_data;

//     custom_robot_data_t custom_robot_data;
//     uint16_t ref_cmd_id;

//     uint8_t first_drawing_flag;
//     uint8_t cur_sending_count;
//     referee_ui_t cur_sending_id;

//     robot_color_t robot_color;

// } Referee_t;

// void referee_hero_draw_marks(Referee_t* ref);
// void referee_general_draw_act_mode(Referee_t* ref);
// void referee_infantry_draw_marks(Referee_t* ref);

#ifdef __cplusplus
}
#endif

#endif /*__ANY_HEADER_H__*/
