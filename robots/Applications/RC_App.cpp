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

#include <algorithm>
#include <cstring>
#include <limits>
#include "apps_classes.hpp"
#include "apps_defines.hpp"
#include "apps_types.hpp"
#include "middleware_types.hpp"
#include "quantize.hpp"
#include "robot_config.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"
#include "uart_isr.hpp"

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
RCApp::RCApp(MW_RTOS::IRTOS& _rtos, mc2::RobotMC& mc_ref,
                         comm::Communication<mc2::RobotMC,
                                                                 mc2::RobotMC::Topics>& communication_ref,
                         IRCComm& rc_comm_ref, isr::uart::UART_ISR& _uart_isr)
        : RTOSApp(_rtos),
            mc(mc_ref),
            communication(communication_ref),
            rc_comm(rc_comm_ref),
            uart_isr(_uart_isr) {}

void RCApp::init() {
    memset(rc_raw.rc_bytes.data(), 0, sizeof(rc_raw.rc_bytes));
    rc_idle_count = 0;

    rc_comm.buffer_init(rc_rx_buffer);
    rc_comm.controller_init(rc.ctrl);
    rc_comm.pc_init(rc.pc);

    rc.control_mode = CTRLER_MODE;
    rc.board_mode = IDLE_MODE;
    rc.board_act_mode = INDPET_MODE;

    pc_board_mode = PATROL_MODE;
    pc_act_mode = INDPET_MODE;
    pc_shoot_mode = SHOOT_CEASE;
    pc_ammo_status = ammo_lid::LidStatus::CLOSED;

    ASSERT(uart_isr.register_routine(
               isr::uart::ECallbacks::RECEIVE_COMPLETE,
               [this](MW_UART::IUART& uart, MW_UART::Peripheral peripheral) {
                   uart_isr_receive_complete(uart, peripheral);
               }),
           "Failed to register UART ISR routine.");

    ASSERT(uart_isr.register_init(
               [this](MW_UART::IUART& uart) { return uart_isr_init(uart); }),
           "Failed to register UART ISR init function.");
}

void RCApp::loop() {
    detect_rc_loss();

    auto message_ts = mc.get_message(rc_raw);

    if (message_ts.has_value()) {
        std::copy(std::begin(rc_raw.rc_bytes), std::end(rc_raw.rc_bytes),
                  rc_rx_buffer.begin());

        parse_raw_rc();
        pub_command_messages();
    }
}

bool RCApp::uart_isr_init(MW_UART::IUART& uart) {
    return uart.receive_data(MW_UART::Peripheral::UART3,
                             uart_rx.rc_bytes.data(), uart_rx.rc_bytes.size());
}

void RCApp::uart_isr_receive_complete(MW_UART::IUART& uart,
                                      MW_UART::Peripheral peripheral) {
    if (peripheral == MW_UART::Peripheral::UART3) {
        mc.pub_message_from_isr(uart_rx);
        uart.receive_data(MW_UART::Peripheral::UART3, uart_rx.rc_bytes.data(),
                          uart_rx.rc_bytes.size());
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
        board_mode = PATROL_MODE;
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
        // TODO: Add assert to fail for unknown switches.
        board_mode = IDLE_MODE;
        act_mode = INDPET_MODE;
        shoot_mode = SHOOT_CEASE;
    }
}

void RCApp::detect_rc_loss() {
    if (rc_idle_count >= 500) {
        send_chassis_command(0, 0, 0, IDLE_MODE, INDPET_MODE);
        send_gimbal_command(0, 0, IDLE_MODE, INDPET_MODE);
    }

    auto message_ts = mc.peek_message(rc_raw);

    if (message_ts.has_value()) {
        rc_idle_count = 0;
    } else {
        rc_idle_count = value_limit(rc_idle_count + 1, 0, 1000);
    }
}

void RCApp::send_gimbal_command(float yaw, float pitch, BoardMode_t board_mode,
                                BoardActMode_t act_mode) {
    mc2::GimbalCommand gimbal_command;
    gimbal_command.delta_yaw = yaw;
    gimbal_command.delta_pitch = pitch;
    gimbal_command.command_bits =
        ((static_cast<uint8_t>(board_mode) & 0x7) << 3) |
        (static_cast<uint8_t>(act_mode) & 0x7);
    communication.transmit_external_message(gimbal_command, simple_comm::NodeID::Chassis, simple_comm::NodeID::Gimbal);
}

void RCApp::send_chassis_command(float v_parallel, float v_perp, float wz,
                                 BoardMode_t board_mode,
                                 BoardActMode_t act_mode) {
    mc2::ChassisCommand chassis_command;
    chassis_command.v_parallel = v_parallel;
    chassis_command.v_perp = v_perp;
    chassis_command.wz = wz;
    chassis_command.command_bits =
        ((static_cast<uint8_t>(board_mode) & 0x7) << 3) |
        (static_cast<uint8_t>(act_mode) & 0x7);

    mc.pub_message(chassis_command);
}

void RCApp::send_shoot_command(ShootActMode_t shoot_mode,
                               ammo_lid::LidStatus ammo_lid_status) {
    mc2::ShootCommand shoot_command;
    shoot_command.command_bits = static_cast<uint8_t>(shoot_mode);

    if (ammo_lid_status == ammo_lid::LidStatus::OPEN) {
        shoot_command.extra_bits = 1;
    } else {
        shoot_command.extra_bits = 0;
    }

    communication.transmit_external_message(shoot_command, simple_comm::NodeID::Chassis, simple_comm::NodeID::Gimbal);
}

void RCApp::pub_command_messages() {
    if (rc.control_mode == PC_MODE) {
        // if (rc.pc.keyboard.F.status == EKeyStatus::PRESSED_TO_RELEASE) {
        //     if (pc_act_mode == GIMBAL_CENTER) {
        //         pc_act_mode = SELF_GYRO;
        //     } else {
        //         pc_act_mode = GIMBAL_CENTER;
        //     }
        // }

        if (rc.pc.keyboard.R.status == EKeyStatus::PRESSED) {
            pc_ammo_status = ammo_lid::LidStatus::OPEN;
        } else {
            pc_ammo_status = ammo_lid::LidStatus::CLOSED;
        }

        if (rc.pc.keyboard.Shift.status == EKeyStatus::PRESSED_TO_RELEASE) {
            // TODO: Enable temporary power uncapping.
        }

        if (rc.pc.mouse.left_click.status == EKeyStatus::PRESSED) {
            pc_shoot_mode = SHOOT_CONT;
        } else {
            pc_shoot_mode = SHOOT_CEASE;
        }

        float v_perp = 0, v_parallel = 0;

        if (rc.pc.keyboard.W.status == EKeyStatus::PRESSED)
            v_parallel += robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.A.status == EKeyStatus::PRESSED)
            v_perp -= robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.S.status == EKeyStatus::PRESSED)
            v_parallel -= robot_config::chassis_params::MAX_TRANSLATION;

        if (rc.pc.keyboard.D.status == EKeyStatus::PRESSED)
            v_perp += robot_config::chassis_params::MAX_TRANSLATION;

        float yaw = in_out_map(rc.pc.mouse.x, -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED,
                               -apps_defines::rc::mouse_max_yaw_magnitude_out,
                               apps_defines::rc::mouse_max_yaw_magnitude_out);

        send_chassis_command(v_parallel, v_perp, yaw, pc_board_mode,
                             pc_act_mode);

        float pitch =
            -in_out_map(rc.pc.mouse.y, -MOUSE_MAX_SPEED, MOUSE_MAX_SPEED,
                        -apps_defines::rc::mouse_max_pitch_magnitude_out,
                        apps_defines::rc::mouse_max_pitch_magnitude_out);

        send_gimbal_command(0, pitch, pc_board_mode, pc_act_mode);
        send_shoot_command(pc_shoot_mode, pc_ammo_status);
    } else {
        BoardMode_t board_mode;
        BoardActMode_t act_mode;
        ShootActMode_t shoot_mode;
        map_switches_to_modes(board_mode, act_mode, shoot_mode);

        float v_perp = in_out_map(
            rc.ctrl.ch2, -apps_defines::rc::joystick_max_offset_magnitude,
            apps_defines::rc::joystick_max_offset_magnitude,
            -robot_config::chassis_params::MAX_TRANSLATION,
            robot_config::chassis_params::MAX_TRANSLATION);
        float v_parallel = in_out_map(
            rc.ctrl.ch3, -apps_defines::rc::joystick_max_offset_magnitude,
            apps_defines::rc::joystick_max_offset_magnitude,
            -robot_config::chassis_params::MAX_TRANSLATION,
            robot_config::chassis_params::MAX_TRANSLATION);
        float wz = in_out_map(rc.ctrl.ch0,
                              -apps_defines::rc::joystick_max_offset_magnitude,
                              apps_defines::rc::joystick_max_offset_magnitude,
                              -robot_config::chassis_params::MAX_ROTATION,
                              robot_config::chassis_params::MAX_ROTATION);

        if (fabs(v_perp) < apps_defines::rc::chassis_joystick_send_threshold)
            v_perp = 0;
        if (fabs(v_parallel) <
            apps_defines::rc::chassis_joystick_send_threshold)
            v_parallel = 0;
        if (fabs(wz) < apps_defines::rc::chassis_joystick_send_threshold)
            wz = 0;

        send_chassis_command(v_parallel, v_perp, wz, board_mode, act_mode);

        float yaw = in_out_map(rc.ctrl.ch0,
                               -apps_defines::rc::joystick_max_offset_magnitude,
                               apps_defines::rc::joystick_max_offset_magnitude,
                               -5.0f * DEGREE2RAD, 5.0f * DEGREE2RAD);
        float pitch = in_out_map(
            rc.ctrl.ch1, -apps_defines::rc::joystick_max_offset_magnitude,
            apps_defines::rc::joystick_max_offset_magnitude, -5.0f * DEGREE2RAD,
            5.0f * DEGREE2RAD);

        // Command delta deadbands.
        if (fabs(yaw) < apps_defines::rc::gimbal_joystick_send_threshold)
            yaw = 0;
        if (fabs(pitch) < apps_defines::rc::gimbal_joystick_send_threshold)
            pitch = 0;

        send_gimbal_command(yaw, pitch, board_mode, act_mode);

        if (rc.ctrl.wheel > 0) {
            send_shoot_command(shoot_mode, ammo_lid::LidStatus::OPEN);
        } else {
            send_shoot_command(shoot_mode, ammo_lid::LidStatus::CLOSED);
        }
    }
}

#endif /*__RC_APP_C__*/
