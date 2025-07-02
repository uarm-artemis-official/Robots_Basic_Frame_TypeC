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
#include <limits>
#include "apps_defines.h"
#include "quantize.hpp"
#include "robot_config.hpp"
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

    pc_board_mode = PATROL_MODE;
    pc_act_mode = GIMBAL_CENTER;
    pc_shoot_mode = SHOOT_CEASE;
    pc_ammo_status = EAmmoLidStatus::CLOSED;
}

void RCApp::loop() {
    detect_rc_loss();

    BaseType_t new_rc_raw_message =
        message_center.get_message(RC_RAW, tmp_rx_buffer, 0);

    if (new_rc_raw_message == pdTRUE) {
        std::copy(std::begin(tmp_rx_buffer), std::end(tmp_rx_buffer),
                  rc_rx_buffer.begin());

        parse_raw_rc();
        pub_command_messages();
    }
}

void RCApp::parse_raw_rc() {
    rc_comm.parse_switches(rc_rx_buffer, rc.ctrl.s1, rc.ctrl.s2);
    if (rc.ctrl.s1 == ESwitchState::DOWN && rc.ctrl.s2 == ESwitchState::DOWN) {
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
        // PC MODE
        board_mode = IDLE_MODE;
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
        board_mode = PATROL_MODE;
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

void RCApp::detect_rc_loss() {
    if (rc_idle_count >= 500) {
        send_chassis_command(0, 0, 0, IDLE_MODE, INDPET_MODE);
        send_gimbal_can_comm(0, 0, IDLE_MODE, INDPET_MODE);
    }

    BaseType_t new_rc_raw_message =
        message_center.peek_message(RC_RAW, tmp_rx_buffer, 0);

    if (new_rc_raw_message == pdTRUE) {
        rc_idle_count = 0;
    } else {
        rc_idle_count = value_limit(rc_idle_count + 1, 0, 1000);
    }
}

void RCApp::send_gimbal_can_comm(float yaw, float pitch, BoardMode_t board_mode,
                                 BoardActMode_t act_mode) {
    int16_t quantized_yaw =
        quantize_float(yaw, -PI, PI, std::numeric_limits<int16_t>::min(),
                       std::numeric_limits<int16_t>::max());
    int16_t quantized_pitch =
        quantize_float(pitch, -PI, PI, std::numeric_limits<int16_t>::min(),
                       std::numeric_limits<int16_t>::max());
    uint32_t command_bits = ((static_cast<uint8_t>(board_mode) & 0x7) << 3) |
                            (static_cast<uint8_t>(act_mode) & 0x7);

    CANCommMessage_t can_comm_message;
    can_comm_message.topic_name = COMMAND_GIMBAL;
    std::memcpy(can_comm_message.data, &quantized_yaw, sizeof(int16_t));
    std::memcpy(&(can_comm_message.data[2]), &quantized_pitch, sizeof(int16_t));
    std::memcpy(&(can_comm_message.data[4]), &command_bits, sizeof(uint32_t));

    message_center.pub_message(COMM_OUT, &can_comm_message);
}

void RCApp::send_chassis_command(float v_parallel, float v_perp, float wz,
                                 BoardMode_t board_mode,
                                 BoardActMode_t act_mode) {
    ChassisCommandMessage_t chassis_command;
    chassis_command.v_parallel = v_parallel;
    chassis_command.v_perp = v_perp;
    chassis_command.wz = wz;
    chassis_command.command_bits =
        ((static_cast<uint8_t>(board_mode) & 0x7) << 3) |
        (static_cast<uint8_t>(act_mode) & 0x7);

    message_center.pub_message(COMMAND_CHASSIS, &chassis_command);
}

void RCApp::send_shoot_command(ShootActMode_t shoot_mode,
                               EAmmoLidStatus ammo_lid_status) {
    ShootCommandMessage_t shoot_command;
    shoot_command.command_bits = static_cast<uint8_t>(shoot_mode);

    if (ammo_lid_status == EAmmoLidStatus::OPEN) {
        shoot_command.extra_bits = 1;
    } else {
        shoot_command.extra_bits = 0;
    }

    CANCommMessage_t can_comm_message;
    can_comm_message.topic_name = COMMAND_SHOOT;
    std::memcpy(&can_comm_message.data, &(shoot_command.command_bits),
                sizeof(uint32_t));
    std::memcpy(&(can_comm_message.data[4]), &(shoot_command.extra_bits),
                sizeof(uint32_t));

    message_center.pub_message(COMM_OUT, &can_comm_message);
}

void RCApp::pub_command_messages() {
    if (rc.control_mode == PC_MODE) {
        if (rc.pc.keyboard.F.status == EKeyStatus::PRESSED_TO_RELEASE) {
            if (pc_act_mode == GIMBAL_CENTER) {
                pc_act_mode = SELF_GYRO;
            } else {
                pc_act_mode = GIMBAL_CENTER;
            }
        }

        if (rc.pc.keyboard.R.status == EKeyStatus::PRESSED) {
            pc_ammo_status = EAmmoLidStatus::OPEN;
        } else {
            pc_ammo_status = EAmmoLidStatus::CLOSED;
        }

        if (rc.pc.keyboard.Shift.status == EKeyStatus::PRESSED_TO_RELEASE) {
            // TODO: Enable temporary power uncapping.
        }

        if (rc.pc.mouse.left_click.status == EKeyStatus::PRESSED) {
            pc_shoot_mode = SHOOT_CONT;
        } else {
            pc_shoot_mode = SHOOT_CEASE;
        }

        float v_perp = 0, v_parallel = 0, wz = 0;

        if (rc.pc.keyboard.W.status == EKeyStatus::PRESSED)
            v_parallel += robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.A.status == EKeyStatus::PRESSED)
            v_perp -= robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.S.status == EKeyStatus::PRESSED)
            v_parallel -= robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.D.status == EKeyStatus::PRESSED)
            v_perp += robot_config::chassis_params::MAX_TRANSLATION;

        send_chassis_command(v_parallel, v_perp, wz, pc_board_mode,
                             pc_act_mode);

        float yaw = in_out_map(rc.pc.mouse.x, -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED,
                               -MAX_MOUSE_YAW_OUT, MAX_MOUSE_YAW_OUT);
        float pitch =
            in_out_map(rc.pc.mouse.y, -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED,
                       -MAX_MOUSE_PITCH_OUT, MAX_MOUSE_PITCH_OUT);

        send_gimbal_can_comm(yaw, pitch, pc_board_mode, pc_act_mode);
        send_shoot_command(pc_shoot_mode, pc_ammo_status);
    } else {
        BoardMode_t board_mode;
        BoardActMode_t act_mode;
        ShootActMode_t shoot_mode;
        map_switches_to_modes(board_mode, act_mode, shoot_mode);

        float v_perp =
            in_out_map(rc.ctrl.ch2, -CHANNEL_OFFSET_MAX_ABS_VAL,
                       CHANNEL_OFFSET_MAX_ABS_VAL,
                       -robot_config::chassis_params::MAX_TRANSLATION,
                       robot_config::chassis_params::MAX_TRANSLATION);
        float v_parallel =
            in_out_map(rc.ctrl.ch3, -CHANNEL_OFFSET_MAX_ABS_VAL,
                       CHANNEL_OFFSET_MAX_ABS_VAL,
                       -robot_config::chassis_params::MAX_TRANSLATION,
                       robot_config::chassis_params::MAX_TRANSLATION);
        float wz = in_out_map(rc.ctrl.ch0, -CHANNEL_OFFSET_MAX_ABS_VAL,
                              CHANNEL_OFFSET_MAX_ABS_VAL,
                              -robot_config::chassis_params::MAX_ROTATION,
                              robot_config::chassis_params::MAX_ROTATION);

        send_chassis_command(v_parallel, v_perp, wz, board_mode, act_mode);

        float yaw = in_out_map(rc.ctrl.ch0, -CHANNEL_OFFSET_MAX_ABS_VAL,
                               CHANNEL_OFFSET_MAX_ABS_VAL, -5.0f * DEGREE2RAD,
                               5.0f * DEGREE2RAD);
        float pitch = in_out_map(rc.ctrl.ch1, -CHANNEL_OFFSET_MAX_ABS_VAL,
                                 CHANNEL_OFFSET_MAX_ABS_VAL, -5.0f * DEGREE2RAD,
                                 5.0f * DEGREE2RAD);

        send_gimbal_can_comm(yaw, pitch, board_mode, act_mode);

        if (rc.ctrl.wheel > 0) {
            send_shoot_command(shoot_mode, EAmmoLidStatus::OPEN);
        } else {
            send_shoot_command(shoot_mode, EAmmoLidStatus::CLOSED);
        }
    }
}

#endif /*__RC_APP_C__*/
