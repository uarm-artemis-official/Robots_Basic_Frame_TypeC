/*******************************************************************************
* @file           : Control_App.c
* @brief          : Recv remote control signal via rc or pc
* @created time	  : Jul, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/* @attention:
 * 	we used circular mode of UART1 DMA to receive remote data in real time
 *  thus we need this task to parse the data (bc the data received constantly, do not
 *  handle it with IT.)*/

#ifndef __RC_APP_C__
#define __RC_APP_C__

#include "RC_App.hpp"
#include <algorithm>
#include <cstring>
#include "apps_defines.h"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

/*********************************************************************************
 *  				  <   GENERAL CTRL OPERATION TABLE  >
 *    @note: <- left dialing /->right dialing /A front dialing /V back dialing
 *    ----------------------------------------------------------------------------
 *    | 						CONTROLLER MODE - Infantry 			     	     |
 *    ----------------------------------------------------------------------------
 *    |		Keys    |    Action   |   				Description  		         |
 *    ----------------------------------------------------------------------------
 *    |  Right Rod  |   <- & ->   | CH0, -&+, Yaw gimbal left&right spin         |
 *    |             |             |           INPDT_MODE: ground left/right spin |
 *    ----------------------------------------------------------------------------
 *    |  Right Rod  |    A & V    | CH1, -&+, Pitch motor's Head down/head up    |
 *    ----------------------------------------------------------------------------
 *    |  Left  Rod  |   <- & ->   | CH2, -&+, INDPT_MODE: ground left/right move |
 *    |             |             |           Gimbal_follow: left/right spin     |
 *    ----------------------------------------------------------------------------
 *    |  Left  Rod  |    A & V    | CH3, -&+, INDPT_MODE: Ground  for/backward   |
 *    |             |             |           other: Follow yaw's direction      |
 *    ----------------------------------------------------------------------------
 *    |  Right Sw   |     UP      | Shoot Continuously    				         |
 *    |    (S2)     |     MID     | Shoot cease fire     					     |
 *    |             |     DOWN    | Self-Gyro(Combo): spin chassis & follow yaw  |
 *    ----------------------------------------------------------------------------
 *    |  Left  Sw   |     UP      | Gimbal Center: yaw axis, chassis chase yaw   |
 *    |    (S1)     |     MID     | IDLE_mode: shut down all system              |
 *    |             |     DOWN    | Gimbal Follow: yaw axis, chassis no spin     |
 *    ----------------------------------------------------------------------------
 *    * Combo SWs:
 *    		S1 Down&S2 Down: Independent mode
 *    		S1 Up  &S2 Down: Self-Gyro mode
 **********************************************************************************
 *    ----------------------------------------------------------------------------
 *    | 							PC MODE - Infantry 		 		     	     |
 *    ----------------------------------------------------------------------------
 *    |	Keys&Mouse  |    Action      |   			Description  		         |
 *    ----------------------------------------------------------------------------
 *    |  	W	  	|  Go forward    |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |  	A	  	|  left panning  |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |	    S       |  go backward   |  All mode       				             |
 *    ----------------------------------------------------------------------------
 *    |  	D	  	|  right panning |  All mode								 |
 *    ----------------------------------------------------------------------------
 *    |  	Q	  	|  left spin 	 |  Only independent mode & follow mode      |
 *    ----------------------------------------------------------------------------
 *    |  	E	  	|  right spin  	 |  Only independent mode & follow mode      |
 *    ----------------------------------------------------------------------------
 *    |  	R	  	|  Reload  	     |  Open lids/Close lids. All mode   		 |
 *    ----------------------------------------------------------------------------
 *    |  	/	  	|  Gear  	     |  Chassis gear manually switch.All mode    |
 *    ----------------------------------------------------------------------------
 *    |  	Ctrl	|  Mode Switch   |  Gyro or Encoder mode  				     |
 *    ----------------------------------------------------------------------------
 *    |  	F		|  Sub Mode Swap |  Gyro mode: Gimbal Center or Self-Gyro    |
 *    |  	 		|  			  	 |  Encoder mode: Gimbal Follow or Ground	 |
 *    ----------------------------------------------------------------------------
 *    |  	/	    |  Accelerate    |  Supercapacitor turned on(long press)/off |
 *    ----------------------------------------------------------------------------
 *    |  	G	  	|  Safety Key    |  IDLE MODE, shut down everything(hold) 	 |
 *    |				|				 |  (release to select the normal mode) 	 |
 *    ----------------------------------------------------------------------------
 *    | Right Click |  Auto aim 	 |  Swap to auto aiming mode (pressed)       |
 *    ----------------------------------------------------------------------------
 *    | Left Click	|  Shoot 	 	 |  Tap to fire a single shot, press and 	 |
 *    |				|				 |	hold to continously shoot.   			 |
 *    ----------------------------------------------------------------------------
 *    |  	B	  	| Mag Reverse    |  Magazine motor reverse to avoid jam		 |
 *    ----------------------------------------------------------------------------
 *
 *    Note: Shift need to be combined with any of WASD keys.
 *
 *********************************************************************************/
RCApp::RCApp(IMessageCenter& message_center_ref, IRCComm& rc_comm_ref)
    : message_center(message_center_ref), rc_comm(rc_comm_ref) {}

void RCApp::init() {
    memset(&tmp_rx_buffer, 0, sizeof(uint8_t) * 18);
    rc_idle_count = 0;

    rc_comm.buffer_init(rc_rx_buffer);
    rc_comm.controller_init(rc.ctrl);
    rc_comm.pc_init(rc.pc);

    rc.control_mode = CTRLER_MODE;
    rc.board_mode = IDLE_MODE;
    rc.board_act_mode = INDPET_MODE;
}

void RCApp::loop() {
    if (rc_idle_count >= 500) {
        uint8_t safe_modes[3] = {IDLE_MODE, INDPET_MODE, SHOOT_CEASE};
        int16_t channels[4] = {0};
        pub_rc_messages(safe_modes, channels);
    }

    BaseType_t new_rc_raw_message =
        message_center.get_message(RC_RAW, tmp_rx_buffer, 0);

    if (new_rc_raw_message == pdTRUE) {
        std::copy(std::begin(tmp_rx_buffer), std::end(tmp_rx_buffer),
                  rc_rx_buffer.begin());

        parse_raw_rc();

        BoardMode_t board_mode;
        BoardActMode_t act_mode;
        ShootActMode_t shoot_mode;
        map_switches_to_modes(board_mode, act_mode, shoot_mode);

        uint8_t modes[3] = {static_cast<uint8_t>(board_mode),
                            static_cast<uint8_t>(act_mode),
                            static_cast<uint8_t>(shoot_mode)};
        int16_t channels[4] = {rc.ctrl.ch0, rc.ctrl.ch1, rc.ctrl.ch2,
                               rc.ctrl.ch3};

        pub_rc_messages(modes, channels);
        rc_idle_count = 0;
    } else {
        rc_idle_count++;
    }
}

void RCApp::parse_raw_rc() {
    rc_comm.parse_switches(rc_rx_buffer, rc.ctrl.s1, rc.ctrl.s2);
    if (rc.ctrl.s1 == ESwitchState::MID && rc.ctrl.s2 == ESwitchState::DOWN) {
        rc.control_mode = PC_MODE;
    } else {
        rc.control_mode = CTRLER_MODE;
    }

    if (rc.control_mode == PC_MODE) {
        rc_comm.parse_pc(rc_rx_buffer, rc.pc);
    } else {
        rc_comm.parse_controller(rc_rx_buffer, rc.ctrl);
    }
}

void RCApp::map_switches_to_modes(BoardMode_t& board_mode,
                                  BoardActMode_t& act_mode,
                                  ShootActMode_t& shoot_mode) {

    if (rc.ctrl.s1 == ESwitchState::DOWN && rc.ctrl.s2 == ESwitchState::DOWN) {
        board_mode = PATROL_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::DOWN &&
               rc.ctrl.s2 == ESwitchState::MID) {
        board_mode = PATROL_MODE;
        act_mode = GIMBAL_FOLLOW;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::DOWN &&
               rc.ctrl.s2 == ESwitchState::UP) {
        board_mode = PATROL_MODE;
        act_mode = GIMBAL_FOLLOW;
        shoot_mode = SHOOT_CONT;
    } else if (rc.ctrl.s1 == ESwitchState::MID &&
               rc.ctrl.s2 == ESwitchState::DOWN) {
        board_mode = AUTO_AIM_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::MID &&
               rc.ctrl.s2 == ESwitchState::MID) {
        board_mode = IDLE_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::MID &&
               rc.ctrl.s2 == ESwitchState::UP) {
        board_mode = IDLE_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CONT;
    } else if (rc.ctrl.s1 == ESwitchState::UP &&
               rc.ctrl.s2 == ESwitchState::DOWN) {
        board_mode = PATROL_MODE;
        act_mode = SELF_GYRO;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::UP &&
               rc.ctrl.s2 == ESwitchState::MID) {
        board_mode = PATROL_MODE;
        act_mode = GIMBAL_CENTER;
        shoot_mode = SHOOT_CEASE;
    } else if (rc.ctrl.s1 == ESwitchState::UP &&
               rc.ctrl.s2 == ESwitchState::UP) {
        board_mode = PATROL_MODE;
        act_mode = GIMBAL_CENTER;
        shoot_mode = SHOOT_CONT;
    } else {
        board_mode = IDLE_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CEASE;
    }
}

void RCApp::pub_rc_messages(uint8_t modes[3], int16_t channels[4]) {
    RCInfoMessage_t rc_info_message;
    memset(&rc_info_message, 0, sizeof(RCInfoMessage_t));
    memcpy(&(rc_info_message.modes), modes, sizeof(uint8_t) * 3);
    memcpy(&(rc_info_message.channels), channels, sizeof(int16_t) * 4);
    message_center.pub_message(RC_INFO, &rc_info_message);

    // Publish to COMM_OUT (Chassis -> Gimbal).
    CANCommMessage_t comm_message;
    memset(&comm_message, 0, sizeof(CANCommMessage_t));
    comm_message.topic_name = RC_INFO;
    memcpy(comm_message.data, modes, sizeof(uint8_t) * 3);
    memcpy(&(comm_message.data[4]), channels, sizeof(int16_t) * 2);
    message_center.pub_message(COMM_OUT, &comm_message);
}

#endif /*__RC_APP_C__*/
