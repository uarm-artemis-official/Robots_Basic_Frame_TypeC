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

#include "string.h"
#include "comms_types.h"

/* functions declaration here */
void queueM_init(QueueManage_t *qm);
void enqueueCanMessage(CAN_TxHeaderTypeDef* header, CanMessage_t *canQueue, QueueManage_t *qm, uint8_t *data);
void sendNextCanMessage(CAN_HandleTypeDef* hcan, CanMessage_t *canQueue, QueueManage_t *qm);

#endif /*__QUEUE_H__*/
