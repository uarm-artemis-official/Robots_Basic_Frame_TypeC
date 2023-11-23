/*******************************************************************************
* @file           : auto_aim_pack.c
* @brief          : parse pack functions for parsing the received packets from
* 					mini PC (JLU version）
* @created time	  : Jul, 2023
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*
*
* This program has been abandoned since we are designing the new auto aiming sys
*******************************************************************************/

#include <auto_aim_pack.h>
#include <string.h>
#include "math.h"

CommVision_t vision_pack;
/**************** For Parsing the Old Vision Pack sent from mini PC *****************/
/*
 * @Brief: init the vision packet
 * @Param: pack: the package received from UART.
 * @Created on: Aug, 2023
 * */
void pack_init(CommVision_t *pack){
	pack->yaw_data   = 0;
	pack->pitch_data = 0;
	pack->dist_data  = 0;
	pack->target_num = 0;
	pack->pack_cond  = PACKCOR;
}

void pack_reset(CommVision_t *pack){
	pack->yaw_data   = 0;
	pack->pitch_data = 0;
	pack->dist_data  = 0;
	pack->target_num = 0;
	pack->pack_cond  = PACKCOR;
}

/*
 * @Brief: parse the packages sent from computer and output the motor data based on data pos
 * @Param: pack: the package received from UART.
 * @Param: pos: The position of the last byte of the currently extracted data, e.g. yaw.
 * @Param: lens: the length of current data, e.g. yaw.
 * @Retval:Parsed data
 * @Author: Haoran,
 * @Created on: Jan, 2022
 */
float parse_pack_indv(char* recv_pack, int pos, int lens){
	char pdata_temp[PACKLEN]; //pack content size + '\0'
	float data = 0;
	memcpy(pdata_temp, recv_pack, PACKLEN);

	//if (pdata_temp[0] == 0xAA){ //check received correct pack head frame， modify here to 0xAA in real world test, 0x41 for tests in 'A'.
		for(int i=0; i<lens; i++){
			data += ((pdata_temp[pos-i-1-2] - '0')*pow(10,i)); // decoding, referring to the vision code.
		}
		if (pdata_temp[pos-lens-2-1]=='0'){
			data=-data;
		}
	return data;
}

/*
 * @Use: parse the packages sent from computer using defined parse indv function
 * @Parameter:pack: the package received from UART
 * @Return: Parsed pack
 * @Author: Wenyuan
 * @Time:May, 2022
 */
CommVision_t parse_all(CommVision_t pack, char* recv_pack){
	//FIXME: need to do CRC verification
	pack.yaw_data   = parse_pack_indv(recv_pack,YAW_POS, DATALEN);
	pack.pitch_data = parse_pack_indv(recv_pack,PITCH_POS,DATALEN);
	pack.dist_data  = parse_pack_indv(recv_pack,DIST_POS,DATALEN);
	pack.target_num = parse_pack_indv(recv_pack,TARGET_POS,DATALEN);
	if(pack.target_num != 0)
		/* objective detected */
		pack.auto_aim_flag = 1;
	else
		pack.auto_aim_flag = 0;
	return pack;
}


