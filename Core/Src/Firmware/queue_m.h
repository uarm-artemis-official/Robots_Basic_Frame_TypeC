/*******************************************************************************
* @file           : queue.h
* @brief          : provide queue management for various comms
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __QUEUE_M_H__
#define __QUEUE_M_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "string.h"

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

/**
  * @brief  queue management main struct
  */
typedef struct {
    int16_t head;  //queue head counter/pointer
    int16_t tail;
    uint16_t queue_size;
} QueueManage_t;

/* can fifo queue */
typedef struct {
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CanMessage_t;

/* functions declaration here */
void queueM_init(QueueManage_t* qm);
void enqueueCanMessage(CAN_TxHeaderTypeDef* header, CanMessage_t* canQueue,
                       QueueManage_t* qm, uint8_t* data);
void sendNextCanMessage(CAN_HandleTypeDef* hcan, CanMessage_t* canQueue,
                        QueueManage_t* qm);

#ifdef __cplusplus
}
#endif

#endif /*__QUEUE_H__*/