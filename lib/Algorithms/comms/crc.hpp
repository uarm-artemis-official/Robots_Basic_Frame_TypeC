/*******************************************************************************
* @file           : CRC.h
* @created time	  : Jul, 2023
* @author(op)     : DJI
*
******************************************************************************
* Copyright (c) 2023 DJI Robomaster.
* All rights reserved.
*******************************************************************************/

#ifndef __INC_CRC8_H__
#define __INC_CRC8_H__

#include "uarm_types.hpp"

#ifndef __FALSE
#define __FALSE 0
unsigned char Get_CRC8_Check_Sum(unsigned char* pchMessage,
                                 unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char* pchMessage,
                                   unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char* pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength,
                             uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);
#endif

#endif /*__INC_CRC8_H__*/
