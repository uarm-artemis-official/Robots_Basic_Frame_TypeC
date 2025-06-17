/*
******************************************************************************
* @file           : Referee_App.h
* @brief      	  : Referee system related files
* @created time	  : Apr, 2024
* @author         : Haoran
*
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
******************************************************************************
*/

#ifndef __REFEREE_APP_H__
#define __REFEREE_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"
#include "referee_data.h"
// #include "main.h"
// #include "stm32f4xx.h"

/* extern global variables here */
// extern uint8_t referee_timeout_check_flag;
// extern uint8_t referee_timeout_counter;

#define REFEREE_NON_RECV_MAX_COUNT \
    100  // maximum count of non-received referee data before reset
// now is 1 second

class RefereeApp : public RTOSApp<RefereeApp> {
   private:
    IMessageCenter& message_center;
    IEventCenter& event_center;
    IDebug& debug;
    IRefUI& ref_ui;  // Referee UI interface

    Referee_t ref;
    uint16_t non_recv_count =
        0;  // Count the number of times referee data is not received

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = REFEREE_TASK_EXEC_TIME;

    RefereeApp(IMessageCenter& msg_center, IEventCenter& evt_center,
               IDebug& debug, IRefUI& ref_ui);
    void init();
    void loop();

    void read_ref_data();
    void draw_all_ui();
    void reset();
};

/* functions declaration here */
// void referee_init(Referee_t* ref);
// void referee_read_data(Referee_t* ref, uint8_t* rx_frame);

// void referee_dma_reset(void);

#endif /*__SRC_REFEREE_APP_C__*/
