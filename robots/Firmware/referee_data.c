/*******************************************************************************
* @file           : referee_data.c
* @brief          : The function to draw referee ui
* @created time	  : May, 2024
* @author         : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __REFEREE_DATA_C__
#define __REFEREE_DATA_C__

#include "referee_data.h"
#include "cmsis_os.h"
#include "string.h"
#include "usart.h"

void referee_transmit_data(uint8_t* data, uint16_t len) {
    /* Transmit data to referee system */
    if (len > 0 && data != NULL) {
        HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
    }
}

// void referee_general_draw_act_mode(Referee_t *ref){
// 	/* Figure examples
// 	 *
// 	 *
// 	 *
// 	 * */
// 	/* Draw 7 letters*/
// 	ref->ui_intrect_data.data_cmd_id = SUB_UI_EXT_CUSTOM_ID;
// 	/* Set up a temp buffer for collect mode */
// 	char buffer[30] = "MODE:GF";
// //	uint8_t *mode = "GF";s
// //	snprintf(buffer, sizeof(buffer), "MODE: %s", mode);

//    ref->ui_custom_data.figure_data_struct.figure_name[0] = INFO_ID;
//    ref->ui_custom_data.figure_data_struct.figure_name[1] = 0;
//    ref->ui_custom_data.figure_data_struct.figure_name[2] = 1;
//    ref->ui_custom_data.figure_data_struct.operate_tpye = 1;
//    ref->ui_custom_data.figure_data_struct.figure_tpye = 7; // String
//    ref->ui_custom_data.figure_data_struct.layer = 0;
//    ref->ui_custom_data.figure_data_struct.details_a = 20;  // Font size
//    ref->ui_custom_data.figure_data_struct.start_x = 1551;
//    ref->ui_custom_data.figure_data_struct.start_y = 770;
//    ref->ui_custom_data.figure_data_struct.color = 1;
//    ref->ui_custom_data.figure_data_struct.details_b = 7;   // String len
//    ref->ui_custom_data.figure_data_struct.width = 2;
//    strcpy((char*)referee.ui_custom_data.data, buffer);
// }

#endif /*__REFEREE_DATA_C__*/
