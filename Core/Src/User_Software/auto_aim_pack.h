/*
******************************************************************************
* @file           : auto_aim_pack.h
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
******************************************************************************
*/


#ifndef __AA_PACK_H__
#define __AA_PACK_H__

#include "comms.h"
#include "main.h"

//define the parse states
#define PACKLEN  32
#define DATALEN  3
#define STATELEN 1
#define PACKCOR 0
#define PACKERR 1
// define the END of each var's pos
#define FCMD_POS 8
#define YAW_POS 16
#define PITCH_POS 24
#define DIST_POS 32
#define TARGET_POS DIST_POS+STATELEN

extern CommVision_t vision_pack;

void pack_init(CommVision_t *pack);
void pack_reset(CommVision_t *pack);
float parse_pack_indv(char* recv_pack, int pos, int lens);
CommVision_t parse_all(CommVision_t pack, char* recv_pack);



#endif /*INC_COMM_H_*/
