/*******************************************************************************
* @file           : redirection.c
* @brief          : redirect the printf to swo or uart for debugging
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/


#ifndef __REDIRECTION_C__
#define __REDIRECTION_C__

//#define __USE_REDIRECTION__

#include <stdio.h>
#include <errno.h>
#include "usart.h"


#ifdef __USE_REDIRECTION__
//#define REDIRECT_SWO 1
#define REDIRECT_UART7 1

extern UART_HandleTypeDef huart7;

#ifdef REDIRECT_SWO
/**
  * @brief     redirect the io flow to swo
  * @retval    return values intro
  */
int _write(int file, char *ptr, int len)
{
	int Idx;
    for(Idx = 0; Idx < len; Idx++){
    	ITM_SendChar(*ptr++);
    }

     return len;
}
#else
/**
  * @brief     redirect the io flow to uart7, no dma, no interrupt
  * @retval    return values intro
  */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart7, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}
#endif
#endif
#else

#endif /*__REDIRECTION_C__*/
