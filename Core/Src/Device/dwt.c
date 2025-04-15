/*******************************************************************************
* @file           : dwt.c
* @brief          : DWT (Data Watchpoint and Trace) high-resolution timer set
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __DWT_C__
#define __DWT_C__

#include "dwt.h"

/**
  * @brief     DWT init function
  * @retval    None
  */
void dwt_init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable Debug Core
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable Cycle Counter
        DWT->CYCCNT = 0; // Reset Cycle Counter Value
    }
}
/**
  * @brief    get timestamp from dwt
  * @retval   time count of dwt
  */
uint32_t dwt_getCnt_us(void){
    return DWT->CYCCNT / SYSTEM_CORE_FREQ;//unit: usec
}


#endif /* __DWT_H__ */
