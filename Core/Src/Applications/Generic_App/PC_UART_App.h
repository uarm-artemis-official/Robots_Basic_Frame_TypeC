/*******************************************************************************
* @file           : PC_UART_App.h
* @brief          : Upper computer communication task
* @restructed     : Mar, 2024
* @maintainer     : James Fu
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __PC_UART_APP_H__
#define __PC_UART_APP_H__

#include <pack_handler.h>
#include "cmsis_os.h"

#include "imu.h"
#include "public_defines.h"
#include "message_center.h"
#include "pack_handler.h"

void PC_UART_Func();

#endif /* SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_ */
