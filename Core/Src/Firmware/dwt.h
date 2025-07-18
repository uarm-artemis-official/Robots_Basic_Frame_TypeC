/*******************************************************************************
* @file           : dwt.h
* @brief          : DWT (Data Watchpoint and Trace) high-resolution timer set
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __DWT_H__
#define __DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f407xx.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"


#define SYSTEM_CORE_FREQ 168 //MHz

/* define main struct of dwt */
typedef struct
{
    uint32_t us;
} DWTTime_t;

/* declare dwt functions */
void dwt_init(void);
uint32_t dwt_getCnt_us(void);
void dwt_sleep(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif
