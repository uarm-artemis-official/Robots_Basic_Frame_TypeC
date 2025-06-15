/*******************************************************************************
* @file           : Control_App.h
* @brief          : remote control task to get rc data from controller or pc
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/*
 * if you copy this file from other OPEN-SOURCE project, please do not
 * add this declaration above. Instead, you are supposed to add their
 * licenses or copyright declaration. Please check ramp.c for more details
 *
 * */

#ifndef __RC_APP_H__
#define __RC_APP_H__

#include "apps_defines.h"
#include "apps_interfaces.h"
#include "apps_types.h"

class RCApp : public RTOSApp<RCApp> {
   private:
    IMessageCenter& message_center;
    IRCComm& rc_comm;

    // TODO: remove and replace with rc_rx_buffer.
    uint8_t tmp_rx_buffer[18];
    Buffer rc_rx_buffer;
    RemoteControl_t rc;
    uint32_t rc_idle_count = 0;

   public:
    static constexpr uint32_t LOOP_PERIOD_MS = RC_TASK_EXEC_TIME;

    explicit RCApp(IMessageCenter& message_center_ref, IRCComm& rc_comm_ref);

    void init();
    void loop();

    void parse_raw_rc();
    void map_switches_to_modes(BoardMode_t& board_mode,
                               BoardActMode_t& act_mode,
                               ShootActMode_t& shoot_mode);
    void pub_rc_messages(uint8_t modes[3], int16_t channels[4]);
    void detect_rc_loss();
    void pub_command_messages();
};

#endif /*__RC_APP_H__*/