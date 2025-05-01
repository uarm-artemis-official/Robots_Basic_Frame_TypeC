/*******************************************************************************
* @file           : feedforward.h
* @brief          : Use feedforward control to improve accuracy of the response of the object
*                   based on input value
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

#include "control_types.h"

/* functions declaration here */
void ff_param_init(FeedForward_t *ff, float kf);
float feedforward(FeedForward_t *ff, float input);

#endif /*__FEEDFORWARD_H__*/
