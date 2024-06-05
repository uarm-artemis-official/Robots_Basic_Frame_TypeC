/*******************************************************************************
* @file           : referee_ui.c
* @brief          : The function to draw referee ui
* @created time	  : May, 2024
* @author         : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __REFEREE_UI_C__
#define __REFEREE_UI_C__

#include "main.h"
#include "string.h"
#include "referee_ui.h"
#include "Chassis_App.h"

Referee_t referee;

extern Chassis_t chassis;

void referee_general_draw_act_mode(Referee_t *ref){
	/* Figure examples
	 *
	 *
	 *
	 * */
	/* Draw 7 letters*/
	ref->ui_intrect_data.data_cmd_id = SUB_UI_EXT_CUSTOM_ID;
	/* Set up a temp buffer for collect mode */
	char buffer[30] = "MODE:GF";
//	uint8_t *mode = "GF";s
//	snprintf(buffer, sizeof(buffer), "MODE: %s", mode);

    ref->ui_custom_data.figure_data_struct.figure_name[0] = INFO_ID;
    ref->ui_custom_data.figure_data_struct.figure_name[1] = 0;
    ref->ui_custom_data.figure_data_struct.figure_name[2] = 1;
    ref->ui_custom_data.figure_data_struct.operate_tpye = 1;
    ref->ui_custom_data.figure_data_struct.figure_tpye = 7; // String
    ref->ui_custom_data.figure_data_struct.layer = 0;
    ref->ui_custom_data.figure_data_struct.details_a = 20;  // Font size
    ref->ui_custom_data.figure_data_struct.start_x = 1551;
    ref->ui_custom_data.figure_data_struct.start_y = 770;
    ref->ui_custom_data.figure_data_struct.color = 1;
    ref->ui_custom_data.figure_data_struct.details_b = 7;   // String len
    ref->ui_custom_data.figure_data_struct.width = 2;
    strcpy((char*)referee.ui_custom_data.data, buffer);
}

void referee_hero_draw_marks(Referee_t *ref){
	/*
	 * Mark examples:
	 *
	 *  ————————|———————— 		// F2
	 * 	  ——————|———————		// F3
	 *      ————|————			// F4
	 *  	  ——|——				// F5
	 *  	    |
	 *  	    F1
	 * */

	/* Figure #1: Vertical line */
	/* Determine the ui drawing times */
	ref->ui_intrect_data.data_cmd_id = SUB_UI_DRAW_7_ID;

	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[0].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[0].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	/*
	 * The top three bytes represent the graphic name, used for graphic indexing,
	   which can be defined by yourself
	 */
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[2] = 0; // Graphic name

	ref->ui_figure_struct_data.interaction_figure[0].figure_tpye= 0;     // Graphic type, 0 is straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[0].layer= 0;           // Number(#) of layers, layer #0
	ref->ui_figure_struct_data.interaction_figure[0].color= 1;           // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[0].width= 3;
	ref->ui_figure_struct_data.interaction_figure[0].start_x= 960; 		 // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[0].start_y= 360; 		 // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[0].details_d = 960; 	 // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[0].details_e = 560; 	 // For straight line, it's end_y

	/* Figure #2: First horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[1].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[1].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[2] = 1;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[1].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[1].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[1].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[1].width= 3;
	ref->ui_figure_struct_data.interaction_figure[1].start_x= 850; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[1].start_y= 540; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[1].details_d = 1070; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[1].details_e = 540;  // For straight line, it's end_y

	/* Figure #3: Third horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[2].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[2].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[2] = 2;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[2].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[2].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[2].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[2].width= 3;
	ref->ui_figure_struct_data.interaction_figure[2].start_x= 870; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[2].start_y= 500; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[2].details_d = 1050; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[2].details_e = 500;  // For straight line, it's end_y

	/* Figure #4: Fourth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[3].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[3].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[2] = 3;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[3].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[3].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[3].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[3].width= 3;
	ref->ui_figure_struct_data.interaction_figure[3].start_x= 890; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[3].start_y= 460; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[3].details_d = 1030; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[3].details_e = 460;  // For straight line, it's end_y

	/* Figure #5: Fifth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[4].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[4].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[2] = 4;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[4].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[4].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[4].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[4].width= 3;
	ref->ui_figure_struct_data.interaction_figure[4].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[4].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[4].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[4].details_e = 420;  // For straight line, it's end_y

	/* Figure #6: sixth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[5].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[5].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[2] = 5;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[5].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[5].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[5].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[5].width= 3;
	ref->ui_figure_struct_data.interaction_figure[5].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[5].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[5].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[5].details_e = 420;  // For straight line, it's end_y

	/* Figure #7: seventh horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[6].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[6].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[2] = 6;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[6].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[6].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[6].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[6].width= 3;
	ref->ui_figure_struct_data.interaction_figure[6].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[6].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[6].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[6].details_e = 420;  // For straight line, it's end_y

	if(ref->first_drawing_flag == 1){
		ref->first_drawing_flag = 1;
	}

	/* Memcpy to main interact struct */
	memcpy(ref->ui_intrect_data.user_data, &ref->ui_figure_struct_data, sizeof(ref->ui_figure_struct_data));

}

void referee_infantry_draw_marks(Referee_t *ref){
	/*
	 * Mark examples:
	 *
	 *  ————————|———————— 		// F2
	 * 	  ——————|———————		// F3
	 *      ————|————			// F4
	 *  	  ——|——				// F5
	 *  	    |
	 *  	    F1
	 * */

	/* Figure #1: Vertical line */
	/* Determine the ui drawing times */
	ref->ui_intrect_data.data_cmd_id = SUB_UI_DRAW_7_ID;

	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[0].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[0].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	/*
	 * The top three bytes represent the graphic name, used for graphic indexing,
	   which can be defined by yourself
	 */
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[0].figure_name[2] = 0; // Graphic name

	ref->ui_figure_struct_data.interaction_figure[0].figure_tpye= 0;     // Graphic type, 0 is straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[0].layer= 0;           // Number(#) of layers, layer #0
	ref->ui_figure_struct_data.interaction_figure[0].color= 1;           // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[0].width= 3;
	ref->ui_figure_struct_data.interaction_figure[0].start_x= 960; 		 // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[0].start_y= 360; 		 // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[0].details_d = 960; 	 // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[0].details_e = 560; 	 // For straight line, it's end_y

	/* Figure #2: First horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[1].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[1].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[1].figure_name[2] = 1;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[1].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[1].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[1].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[1].width= 3;
	ref->ui_figure_struct_data.interaction_figure[1].start_x= 850; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[1].start_y= 540; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[1].details_d = 1070; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[1].details_e = 540;  // For straight line, it's end_y

	/* Figure #3: Third horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[2].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[2].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[2].figure_name[2] = 2;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[2].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[2].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[2].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[2].width= 3;
	ref->ui_figure_struct_data.interaction_figure[2].start_x= 870; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[2].start_y= 500; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[2].details_d = 1050; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[2].details_e = 500;  // For straight line, it's end_y

	/* Figure #4: Fourth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[3].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[3].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[3].figure_name[2] = 3;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[3].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[3].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[3].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[3].width= 3;
	ref->ui_figure_struct_data.interaction_figure[3].start_x= 890; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[3].start_y= 460; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[3].details_d = 1030; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[3].details_e = 460;  // For straight line, it's end_y

	/* Figure #5: Fifth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[4].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[4].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[4].figure_name[2] = 4;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[4].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[4].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[4].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[4].width= 3;
	ref->ui_figure_struct_data.interaction_figure[4].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[4].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[4].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[4].details_e = 420;  // For straight line, it's end_y

	/* Figure #6: sixth horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[5].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[5].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[5].figure_name[2] = 5;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[5].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[5].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[5].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[5].width= 3;
	ref->ui_figure_struct_data.interaction_figure[5].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[5].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[5].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[5].details_e = 420;  // For straight line, it's end_y

	/* Figure #7: seventh horizontal line */
	/* Determine the ui drawing times */
	if(ref->first_drawing_flag)
		ref->ui_figure_struct_data.interaction_figure[6].operate_tpye= 1; // 0: None, 1: Add, 2: Modify. 3: Delete
	else
		ref->ui_figure_struct_data.interaction_figure[6].operate_tpye= 2; // If not first drawing, select modify
	/* Drawing */
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[0] = FIGURE_ID;
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[1] = 0;
	ref->ui_figure_struct_data.interaction_figure[6].figure_name[2] = 6;// Graphic name
	ref->ui_figure_struct_data.interaction_figure[6].figure_tpye= 0;  // Graphic type, 0: straight line, check user manual for others
	ref->ui_figure_struct_data.interaction_figure[6].layer= 0; // Number of layers
	ref->ui_figure_struct_data.interaction_figure[6].color= 1; // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
	ref->ui_figure_struct_data.interaction_figure[6].width= 3;
	ref->ui_figure_struct_data.interaction_figure[6].start_x= 910; // Start point/origin's x-coordinate
	ref->ui_figure_struct_data.interaction_figure[6].start_y= 420; // Start point/origin's y-coordinate
	ref->ui_figure_struct_data.interaction_figure[6].details_d = 1010; // For straight line, it's end_x
	ref->ui_figure_struct_data.interaction_figure[6].details_e = 420;  // For straight line, it's end_y

	if(ref->first_drawing_flag == 1){
		ref->first_drawing_flag = 1;
	}

	/* Memcpy to main interact struct */
	memcpy(ref->ui_intrect_data.user_data, &ref->ui_figure_struct_data, sizeof(ref->ui_figure_struct_data));

}



#endif /*__REFEREE_UI_C__*/
