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
        DWT->CYCCNT = 0; // Reset Cycle Counter Value
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable Cycle Counter
    }
}
/**
  * @brief    get timestamp from dwt
  * @retval   time count of dwt
  */
uint32_t dwt_getCnt_us(void){
    return DWT->CYCCNT / SYSTEM_CORE_FREQ;//unit: usec
}

// Suitable sleep function for when all interrupts are disabled.
// This function relies on counting CPU clock cycles using DWT.
// This is fairly accurate for times < 1000 ms but loses accuracy
// for longer sleep durations.
//
// HAL_Delay and FreeRTOS delay functions rely on interrupts to functions.
// In critical sections or when __disable_irq() is called, these functions
// will hang when called.
void dwt_sleep(uint32_t ms) {
	SystemCoreClockUpdate(); // update core clock variable
	uint64_t cycles = (SystemCoreClock / 1000L);
	uint32_t count = 0;
	while (count < ms) {
		uint64_t start = DWT->CYCCNT;
		while (((uint64_t) DWT->CYCCNT - start + 0xffffffff % 0xffffffff) < cycles);
		count++;
	}
}

#endif /* __DWT_H__ */
