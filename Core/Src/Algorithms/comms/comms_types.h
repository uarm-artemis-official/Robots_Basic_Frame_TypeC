#ifndef __COMMS_TYPES_H
#define __COMMS_TYPES_H

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

/* =========================================================================
 * QUEUE M TYPES
 * ====================================================================== */
/**
  * @brief  queue management main struct
  */
typedef struct{
	int16_t head; //queue head counter/pointer
	int16_t tail;
}QueueManage_t;

/* can fifo queue */
typedef struct
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CanMessage_t;

#endif