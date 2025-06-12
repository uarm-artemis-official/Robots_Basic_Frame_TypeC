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

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class PCUARTApp : public RTOSApp<PCUARTApp> {
   private:
    uint8_t new_pack_buffer[64];  // TODO: Make same as MAX_PACK_BUFFER_SIZE.
    IMessageCenter& message_center;
    uint32_t idle_count = 0;
    float recent_deltas[2];

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = PC_UART_TASK_EXEC_TIME;
    PCUARTApp(IMessageCenter& message_center_ref);
    void init();
    void loop();
};

#endif /* SRC_APPLICATIONS_GENERIC_APP_PC_UART_APP_H_ */