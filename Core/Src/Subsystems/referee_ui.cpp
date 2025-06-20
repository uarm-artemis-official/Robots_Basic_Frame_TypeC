#ifndef SRC_REF_UI_CPP_
#define SRC_REF_UI_CPP_

#include "referee_ui.h"
#include "crc.h"
#include "referee_data.h"
#include "string.h"

void RefereeUI::init() {
    // TODO: Implement init
    // memset(&(ref_ui.ui_figure_struct_data), 0,
    //        sizeof(robot_interaction_data_t));
    memset(&(ref_ui.ui_intrect_data), 0, sizeof(robot_interaction_data_t));
    memset(&(ref_ui.ui_draw_marks_data), 0, sizeof(interaction_figure_7_t));
    memset(&(ref_ui.ui_draw_info_data), 0, sizeof(interaction_figure_7_t));

    ref_ui.ui_intrect_data.data_cmd_id = UI_IDLE_ID;
    ref_ui.cur_sending_id = UI_TYPE_IDLE_ID;
    ref_ui.first_drawing_flag = true;
    ref_ui.first_drawing_flag_info = true;
    ref_ui.cur_sending_count = 0;
    ref_ui.pack_seq = 0;
}

static uint8_t test_data = 0;

void RefereeUI::set_ui_data(referee_ui_type_t ui_type, uint8_t robot_id) {
    if (robot_id == 3 || robot_id == 4 || robot_id == 103 ||
        robot_id == 104) {  // Infantry
        ref_ui.ui_intrect_data.sender_id = robot_id;
        switch (robot_id) {
            // TODO: change to use the actual receiver id we need
            case 3:
                ref_ui.ui_intrect_data.receiver_id = 0x0103;
                break;  // Red team #3 infantry client
            case 4:
                ref_ui.ui_intrect_data.receiver_id = 0x0104;
                break;  // Red team #4 infantry client
            case 103:
                ref_ui.ui_intrect_data.receiver_id = 0x0167;
                break;  // Blue team #3 infantry client
            case 104:
                ref_ui.ui_intrect_data.receiver_id = 0x0168;
                break;  // Blue team #4 infantry client
        }

        switch (ui_type) {
            case UI_ROBOT_VAILD_INFO:
                draw_vaild_info();
                break;
            /* Hero Draw marks */
            case UI_INFANTRY_MARK || UI_HERO_MARK:
                draw_marks();
                break;
            default:
                break;
        }
    } else if (robot_id == 1 || robot_id == 101) {  // Hero
        /* Set up */
        ref_ui.ui_intrect_data.sender_id = robot_id;
        /* Set up client ID */
        // TODO: change to use the actual receiver id we need
        switch (robot_id) {
            case 1:
                ref_ui.ui_intrect_data.receiver_id = 0x0101;
                break;  // Red team #1 Hero client
            case 101:
                ref_ui.ui_intrect_data.receiver_id = 0x0165;
                break;  // Blue team #1 Hero client
        }

        switch (ui_type) {
            case UI_ROBOT_VAILD_INFO:
                draw_vaild_info();
                break;
            /* Hero Draw marks */
            case UI_HERO_MARK || UI_INFANTRY_MARK:
                draw_marks();
                break;
            default:
                break;
        }
    }
}

void RefereeUI::send_ui_data(uint16_t cmd_id, uint16_t len) {
    uint16_t frame_length =
        HEADER_LEN + CMD_LEN + len + CRC_LEN;  // length of data frame

    memset(ref_tx_frame, 0, frame_length);  // Refresh the tx data buffer

    /* Packing the tx header */
    ref_tx_frame[0] = SOF_ID;  // First byte of the data pool, 1 byte
    memcpy(&ref_tx_frame[1], (uint8_t*) &len,
           sizeof(len));  // Actual data len in the data pool, 2 bytes
    ref_tx_frame[3] = ref_ui.pack_seq;  // Pack sequence, start from 3, 1 byte
    Append_CRC8_Check_Sum(
        ref_tx_frame,
        HEADER_LEN);  // CRC8 Calibration required by referee system

    /* Pack cmd id */
    memcpy(&ref_tx_frame[HEADER_LEN], (uint8_t*) &cmd_id, CMD_LEN);

    /* Pack actual data frame */
    memcpy(&ref_tx_frame[HEADER_LEN + CMD_LEN],
           (uint8_t*) &ref_ui.ui_intrect_data, len);
    Append_CRC16_Check_Sum(ref_tx_frame, frame_length);  // CRC16 Calibration

    if (ref_ui.pack_seq == 0xff)
        ref_ui.pack_seq = 0;
    else
        ref_ui.pack_seq++;

    // HAL_UART_Transmit_DMA(&huart1, ref_tx_frame, frame_length);
    referee_transmit_data(ref_tx_frame, frame_length);
}

void RefereeUI::draw_marks() {
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
    ref_ui.ui_intrect_data.data_cmd_id = SUB_UI_DRAW_7_ID;

    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[0].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[0].operate_tpye =
            2;  // If not first drawing, select modify
                /* Drawing */
                /*
	 * The top three bytes represent the graphic name, used for graphic indexing,
	   which can be defined by yourself
	 */
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[2] =
        0;  // Graphic name

    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_tpye =
        0;  // Graphic type, 0 is straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[0].layer =
        0;  // Number(#) of layers, layer #0
    ref_ui.ui_draw_marks_data.interaction_figure[0].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[0].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[0].start_x =
        960;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[0].start_y =
        360;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[0].details_d =
        960;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[0].details_e =
        560;  // For straight line, it's end_y

    /* Figure #2: First horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[1].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[1].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[2] =
        1;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[1].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[1].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[1].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[1].start_x =
        850;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[1].start_y =
        540;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[1].details_d =
        1070;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[1].details_e =
        540;  // For straight line, it's end_y

    /* Figure #3: Third horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[2].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[2].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[2] =
        2;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[2].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[2].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[2].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[2].start_x =
        870;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[2].start_y =
        500;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[2].details_d =
        1050;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[2].details_e =
        500;  // For straight line, it's end_y

    /* Figure #4: Fourth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[3].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[3].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[2] =
        3;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[3].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[3].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[3].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[3].start_x =
        890;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[3].start_y =
        460;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[3].details_d =
        1030;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[3].details_e =
        460;  // For straight line, it's end_y

    /* Figure #5: Fifth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[4].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[4].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[2] =
        4;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[4].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[4].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[4].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[4].start_x =
        910;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[4].start_y =
        420;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[4].details_d =
        1010;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[4].details_e =
        420;  // For straight line, it's end_y

    /* Figure #6: sixth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[5].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[5].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[2] =
        5;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[5].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[5].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[5].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[5].start_x =
        910;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[5].start_y =
        420;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[5].details_d =
        1010;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[5].details_e =
        420;  // For straight line, it's end_y

    /* Figure #7: seventh horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag)
        ref_ui.ui_draw_marks_data.interaction_figure[6].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[6].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[6].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[6].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[6].figure_name[2] =
        6;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[6].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[6].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[6].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[6].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[6].start_x =
        910;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[6].start_y =
        420;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[6].details_d =
        1010;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[6].details_e =
        420;  // For straight line, it's end_y

    if (ref_ui.first_drawing_flag == 1) {
        ref_ui.first_drawing_flag = 1;
    }

    /* Memcpy to main interact struct */
    memcpy(ref_ui.ui_intrect_data.user_data, &ref_ui.ui_draw_marks_data,
           sizeof(ref_ui.ui_draw_marks_data));
}

void RefereeUI::draw_vaild_info() {
    // Draw valid info on the UI includes:
    // act mode, level

    /* Figure #1: Vertical line */
    /* Determine the ui drawing times */
    ref_ui.ui_intrect_data.data_cmd_id = SUB_UI_DRAW_7_ID;

    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[0].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[0].operate_tpye =
            2;  // If not first drawing, select modify
                /* Drawing */
                /*
	 * The top three bytes represent the graphic name, used for graphic indexing,
	   which can be defined by yourself
	 */
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_name[2] =
        0;  // Graphic name

    ref_ui.ui_draw_marks_data.interaction_figure[0].figure_tpye =
        0;  // Graphic type, 0 is straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[0].layer =
        0;  // Number(#) of layers, layer #0
    ref_ui.ui_draw_marks_data.interaction_figure[0].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[0].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[0].start_x =
        960;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[0].start_y =
        360;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[0].details_d =
        960;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[0].details_e =
        560;  // For straight line, it's end_y

    /* Figure #2: First horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[1].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[1].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_name[2] =
        1;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[1].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[1].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[1].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[1].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[1].start_x =
        850;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[1].start_y =
        540;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[1].details_d =
        1070;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[1].details_e =
        540;  // For straight line, it's end_y

    /* Figure #3: Third horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[2].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[2].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_name[2] =
        2;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[2].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[2].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[2].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[2].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[2].start_x =
        870;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[2].start_y =
        500;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[2].details_d =
        1050;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[2].details_e =
        500;  // For straight line, it's end_y

    /* Figure #4: Fourth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[3].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[3].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_name[2] =
        3;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[3].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[3].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[3].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[3].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[3].start_x =
        890;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[3].start_y =
        460;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[3].details_d =
        1030;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[3].details_e =
        460;  // For straight line, it's end_y

    /* Figure #5: Fifth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[4].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[4].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_name[2] =
        4;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[4].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[4].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[4].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[4].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[4].start_x =
        910;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[4].start_y =
        420;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[4].details_d =
        1010;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[4].details_e =
        420;  // For straight line, it's end_y

    /* Figure #6: sixth horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_draw_marks_data.interaction_figure[5].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_draw_marks_data.interaction_figure[5].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[0] = FIGURE_ID;
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[1] = 0;
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_name[2] =
        5;  // Graphic name
    ref_ui.ui_draw_marks_data.interaction_figure[5].figure_tpye =
        0;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_draw_marks_data.interaction_figure[5].layer =
        0;  // Number of layers
    ref_ui.ui_draw_marks_data.interaction_figure[5].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_draw_marks_data.interaction_figure[5].width = 3;
    ref_ui.ui_draw_marks_data.interaction_figure[5].start_x =
        910;  // Start point/origin's x-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[5].start_y =
        420;  // Start point/origin's y-coordinate
    ref_ui.ui_draw_marks_data.interaction_figure[5].details_d =
        1010;  // For straight line, it's end_x
    ref_ui.ui_draw_marks_data.interaction_figure[5].details_e =
        420;  // For straight line, it's end_y

    /* Figure #7: seventh horizontal line */
    /* Determine the ui drawing times */
    if (ref_ui.first_drawing_flag_info)
        ref_ui.ui_figure_struct_data.interaction_figure[6].operate_tpye =
            1;  // 0: None, 1: Add, 2: Modify. 3: Delete
    else
        ref_ui.ui_figure_struct_data.interaction_figure[6].operate_tpye =
            2;  // If not first drawing, select modify
    /* Drawing */
    ref_ui.ui_figure_struct_data.interaction_figure[6].figure_name[0] =
        FIGURE_ID;
    ref_ui.ui_figure_struct_data.interaction_figure[6].figure_name[1] = 1;
    ref_ui.ui_figure_struct_data.interaction_figure[6].figure_name[2] =
        6;  // Graphic name
    ref_ui.ui_figure_struct_data.interaction_figure[6].figure_tpye =
        6;  // Graphic type, 0: straight line, check user manual for others
    ref_ui.ui_figure_struct_data.interaction_figure[6].layer =
        0;  // Number of layers
    ref_ui.ui_figure_struct_data.interaction_figure[6].color =
        1;  // 0 Correspond team color(Red/Blue), 1 yellow, 2 green
    ref_ui.ui_figure_struct_data.interaction_figure[6].width = 3;
    ref_ui.ui_figure_struct_data.interaction_figure[6].start_x = 1500;
    ref_ui.ui_figure_struct_data.interaction_figure[6].start_y = 800;
    ref_ui.ui_figure_struct_data.interaction_figure[6].details_c =
        test_data; // act mode
    ref_ui.ui_figure_struct_data.interaction_figure[6].details_d = 0;
    ref_ui.ui_figure_struct_data.interaction_figure[6].details_e = 0;

    test_data++;
    if (test_data > 9) {
        test_data = 0;
    }

    if (ref_ui.first_drawing_flag_info == 1) {
        ref_ui.first_drawing_flag_info = 0;
    } 
    // else {
    //     ref_ui.first_drawing_flag_info = 1;
    // }

    /* Memcpy to main interact struct */
    memcpy(ref_ui.ui_intrect_data.user_data, &ref_ui.ui_draw_marks_data,
           sizeof(ref_ui.ui_draw_marks_data));
}

#endif /* SRC_REF_UI_CPP_ */
