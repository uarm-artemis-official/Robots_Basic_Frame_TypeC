/*******************************************************************************
* @file           : queue.c
* @brief          : provide queue management for various comms
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __QUEUE_M_C__
#define __QUEUE_M_C__

#include <queue_m.h>
#include "main.h"
#include "string.h"

/* Since we have multiple can comm works in the future , there is necessity that apply
 * FIFO Queue management of our CAN2 data pool. */
/***************************** CAN COMM QUEUE ************************************/
void queueM_init(QueueManage_t *qm){
	qm->head = 0;
	qm->tail = 0;
}

/**
  * @brief     can comms enqueue
  * @param[in] header: can type header
  * @param[in] data: the data would be transmitted
  * @retval    None
  */
void enqueueCanMessage(CAN_TxHeaderTypeDef* header, CanMessage_t *canQueue, QueueManage_t *qm, uint8_t *data){
    if ((qm->tail + 1) % MAX_QUEUE_SIZE == qm->head)
    {
        /* Queue is full, cannot enqueue message */
        return;
    }
    /* follow fifo rules */
    canQueue[qm->tail].header = *header;
    memcpy(canQueue[qm->tail].data, data, 8);
    /* tail ++ */
    qm->tail = (qm->tail + 1) % MAX_QUEUE_SIZE;
}

/**
  * @brief     can comm send data from queue
  * @param[in] header: can type header
  * @retval    None
  */
void sendNextCanMessage(CAN_HandleTypeDef* hcan, CanMessage_t *canQueue, QueueManage_t *qm){
    if (qm->head == qm->tail)
    {
        /* Queue is empty, nothing to send */
        return;
    }
    uint32_t mailbox;
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &(canQueue[qm->head].header), canQueue[qm->head].data, &mailbox);
    if (status == HAL_OK)
    {
        /* Message has been added to the mailbox successfully, remove it from the queue */
    	qm->head = (qm->head + 1) % MAX_QUEUE_SIZE;
    }
}

#endif /*__QUEUE_C__*/
