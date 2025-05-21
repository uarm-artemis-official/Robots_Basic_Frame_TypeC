/*******************************************************************************
* @file           : Chassis_App.c
* @brief          : chassis task managing 4 chassis motors
* @restructed     : Jul, 2023
* @maintainer     : Haoran, AzureRin
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/
#ifndef __CHASSIS_APP_C__
#define __CHASSIS_APP_C__

#define WITH_SLIPRING
//#define CHASSIS_POWER_LIMIT

#include "Chassis_App.h"
#include "apps_defines.h"
#include "debug.h"
#include "message_center.h"
#include "pid.h"
#include "public_defines.h"
#include "uarm_lib.h"
#include "uarm_math.h"
#include "uarm_os.h"

ChassisApp::ChassisApp(IMessageCenter& message_center_ref, IDebug& debug_ref)
    : message_center(message_center_ref), debug(debug_ref) {}

void ChassisApp::init() {
    debug.set_led_state(RED, ON);

    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
        memset(&(motor_controls[i]), 0, sizeof(Chassis_Wheel_Control_t));
        pid_param_init(&(motor_controls[i].f_pid), max_out_wheel,
                       max_I_out_wheel, max_err_wheel, kp_wheel, ki_wheel,
                       kd_wheel);
    }

    motor_controls[0].stdid = CHASSIS_WHEEL1;
    motor_controls[1].stdid = CHASSIS_WHEEL2;
    motor_controls[2].stdid = CHASSIS_WHEEL3;
    motor_controls[3].stdid = CHASSIS_WHEEL4;

    pid_param_init(&(chassis.f_pid), 8000, 500, 5000, 600, 0.1,
                   100);  // chassis twist pid init
    /* set initial chassis mode to idle mode or debug mode */
    chassis.chassis_mode = IDLE_MODE;
    chassis.chassis_act_mode = INDPET_MODE;

    chassis.chassis_gear_mode = AUTO_GEAR;

    set_initial_state();
}

void ChassisApp::set_initial_state() {
    chassis.vx = 0;
    chassis.vy = 0;
    chassis.wz = 0;

    chassis.gimbal_axis.vx = 0;
    chassis.gimbal_axis.vy = 0;
    chassis.gimbal_axis.wz = 0;
    chassis.gimbal_yaw_rel_angle = 0;
    chassis.gimbal_yaw_abs_angle = 0;

    memset(&(chassis.gimbal_axis), 0, sizeof(Gimbal_Axis_t));
    memset(&(chassis.ref_power_stat), 0, sizeof(ChassisPowerStat_t));

    chassis.prev_robot_level = 1;  // Initalized to level 1
    chassis.cur_robot_level = 1;
    select_chassis_speed(chassis.prev_robot_level);

    /* reset mecanum wheel speed */
    for (int i = 0; i < 4; i++)
        chassis.mec_spd[i] = 0;

    memset(rc_channels, 0, sizeof(int16_t) * 4);
}

void ChassisApp::loop() {
    chassis_get_rc_info(rc_channels);
    chassis_get_wheel_feedback();
    chassis_get_gimbal_rel_angles();

    chassis_update_chassis_coord(rc_channels);
    chassis_update_gimbal_coord(rc_channels);
    // chassis_manual_gear_set(&chassis, &rc);
    chassis_exec_act_mode();
    chassis_calc_wheel_pid_out();
    chassis_send_wheel_volts();
}

/*
 * @brief 	  Inversely kinematics of swerve
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void ChassisApp::swerve_wheel_decomp() {
    /* Assume wheels are calibriated to forward position (i.e. forward is 0 degrees)
     * These calculations will be in degrees for MG4005 since 3600 dps is 360 degrees
	 *			 
	 *		 v1  [] ---- [] v2     <Front>		   	 A      __		      0/360 deg
	 *		      |      |					         | vy  /		 	    O  
	 *		  	  |	     |                           |     \__>   wz       180  deg  
	 *		 v4	 [] ---- [] v3     <Rear>              ----> vx  				 
	 *										
     Annotations used for velocity vector of robot:
        V : Translational velocity vector,
        Vx, Vy : translational velocity vector component along X and Y axis respectively,
        vx, vy : magnitude of translational velocity component along X and Y axis respectively,
        Ω : angular (rotational) velocity vector, it is positive in counterclockwise direction,
        wz : magnitude of angular velocity vector,
        R : the distance vector from the axis of rotation to swerve module,
        X : The distance from wheel center of front and rear wheel along X axis,
        Y : The distance from wheel center of left and right wheel along Y axis.

	Annotations used for swerve module:
        Vi(i = 1, 2, 3, 4)  : resultant velocity vector,
        Vit(i = 1, 2, 3, 4) : translational velocity vector,
        Vir(i = 1, 2, 3, 4) : rotational velocity vector,
        vi(i = 1, 2, 3, 4)  : resultant velocity,
        vix(i = 1, 2, 3, 4) : velocity component along X axis,
        viy(i = 1, 2, 3, 4) : velocity component along Y axis,
        0i : Angle of azimuth motor

     The general inverse kinematic equation is,
        Vi = Vit + Vir (1)
        V1 = V + Ω × R (2)
        V1x = Vx + (Ω × R)x (3)
        V1y = Vy + (Ω × R)y (3)
        V1x = Vx - Ω ∗ X/2 (4)
        V1y = Vy + Ω ∗ Y/2 (4)
    
    For module 1,
        v1x = vx - wz ∗ X/2 
        v1y = vy + wz ∗ Y/2 
        v1 = sqrt((v1x)^2 + (v1y)^2)) 
        θ1 = atan2(v1y,v1x) * (1800/pi)
    
    For module 2,
        v2x = vx + wz ∗ X/2 
        v2y = vy + wz ∗ Y/2 
        v2 = sqrt((v2x)^2 + (v2y)^2)) 
        θ2 = atan2(v2y,v2x) * (1800/pi)

    For module 3,
        v3x = vx + wz ∗ X/2 
        v3y = vy - wz ∗ Y/2 
        v3 = sqrt((v3x)^2 + (v3y)^2)) 
        θ3 = atan2(v3y,v3x) * (1800/pi)

    For module 4,
        v4x = vx - wz ∗ X/2 
        v4y = vy - wz ∗ Y/2 
        v4 = sqrt((v4x)^2 + (v4y)^2)) 
        θ4 = atan2(v4y,v4x) * (1800/pi)
    */
    /* Define variables shown above */
    // float v1x =
    //     chassis_hdlr->vx - chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH * 0.5);
    // float v1y =
    //     chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_Y_LENGTH * 0.5);
    // float v2x =
    //     chassis_hdlr->vx + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH * 0.5);
    // float v2y =
    //     chassis_hdlr->vy + chassis_hdlr->wz * (CHASSIS_WHEEL_Y_LENGTH * 0.5);
    // float v3x =
    //     chassis_hdlr->vx + chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH * 0.5);
    // float v3y =
    //     chassis_hdlr->vy - chassis_hdlr->wz * (CHASSIS_WHEEL_Y_LENGTH * 0.5);
    // float v4x =
    //     chassis_hdlr->vx - chassis_hdlr->wz * (CHASSIS_WHEEL_X_LENGTH * 0.5);
    // float v4y =
    //     chassis_hdlr->vy - chassis_hdlr->wz * (CHASSIS_WHEEL_Y_LENGTH * 0.5);

    // /* For speed (Vi) control of M3508 */
    // chassis_hdlr->mec_spd[CHASSIS_WHEEL1_CAN_ID] =
    //     (int16_t) (sqrt(pow(v1x, 2) + pow(v1y, 2))) * CHASSIS_MOTOR_DEC_RATIO;
    // chassis_hdlr->mec_spd[CHASSIS_WHEEL2_CAN_ID] =
    //     (int16_t) (sqrt(pow(v2x, 2) + pow(v2y, 2))) *
    //     CHASSIS_MOTOR_DEC_RATIO chassis_hdlr->mec_spd[CHASSIS_WHEEL3_CAN_ID] =
    //         (int16_t) (sqrt(pow(v3x, 2) + pow(v3y, 2))) *
    //         CHASSIS_MOTOR_DEC_RATIO;
    // chassis_hdlr->mec_spd[CHASSIS_WHEEL4_CAN_ID] =
    //     (int16_t) (sqrt(pow(v4x, 2) + pow(v4y, 2))) * CHASSIS_MOTOR_DEC_RATIO;
    // /* Zero-ing of MG4005 (only needed if motors are not pointing in the "forward" direction)*/

    // /* Set max rotaional speed (Ω) of MG4005 in dps */
    // chassis_hdlr->swerve_speed[SWERVE_1_CAN_ID] =
    //     (uint16_t) (9000)

    //     /* For angle (0i) control of MG4005 [-1800,1800] */
    //     chassis_hdlr->swerve_angle[SWERVE_1_CAN_ID] =
    //         (uint32_t) (atan2(v1y, v1x) *
    //                     (1800 /
    //                      pi)) if chassis_hdlr->swerve_angle[SWERVE_1_CAN_ID]
}

int32_t pack_lk_motor_message(bool spin_cw, uint16_t max_speed,
                              uint32_t angle) {
    ASSERT(angle < 40000, "LK angle cannot be greater than 36000");
    int32_t motor_message_value = 0;
    motor_message_value |= (((uint32_t) max_speed) & 0x0fff) << 16;
    motor_message_value |= angle & 0xffff;

    if (spin_cw) {
        motor_message_value *= -1;
    }
    return motor_message_value;
}

/*
 * @brief 	  Inversely calculate the mecanum wheel speed
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void ChassisApp::mecanum_wheel_calc_speed() {
    /* Assume we install the mecanum wheels as O type (also have X type), right hand define positive dir
	 *			 x length
	 *		 v1  \\ -- //  v2     <Front>		   	 A      __			wheels define:
	 *		      |    |		y length			 | vy  /		 	 		  ___
	 *		  	  |	   |                             |     \__>   wz    \\ ->    | \ |
	 *		 v4	 // -- \\  v3     <Rear>             -----> vx  				 | \ |
	 *																			 |___|
	 *		--	vector([vx, vy, wz]) --
	 *	v1	=  [ vx,  vy,   wz] * (rx + ry) * motor_gearbox_ratio ->rad/s
	 *	v2  =  [-vx,  vy,  -wz] * (rx + ry) * motor_gearbox_ratio
	 *	v3  =  [ vx,  vy,  -wz] * (rx + ry) * motor_gearbox_ratio
	 * 	v4  =  [-vx,  vy,   wz] * (rx + ry) * motor_gearbox_ratio
	 *
	 * */
    /* X type installation */
    chassis.mec_spd[CHASSIS_WHEEL1_CAN_ID] =
        (int16_t) (chassis.vx + chassis.vy +
                   chassis.wz *
                       (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) *
                       0.5) *
        CHASSIS_MOTOR_DEC_RATIO;
    chassis.mec_spd
        [CHASSIS_WHEEL2_CAN_ID] = /* We will put a negative infront of the eq. as motor install is flipped*/
        (int16_t) -(-chassis.vx + chassis.vy -
                    chassis.wz *
                        (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) *
                        0.5) *
        CHASSIS_MOTOR_DEC_RATIO;
    chassis.mec_spd
        [CHASSIS_WHEEL3_CAN_ID] = /* We will put a negative infront of the eq. as motor install is flipped*/
        (int16_t) -(chassis.vx + chassis.vy -
                    chassis.wz *
                        (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) *
                        0.5) *
        CHASSIS_MOTOR_DEC_RATIO;
    chassis.mec_spd[CHASSIS_WHEEL4_CAN_ID] =
        (int16_t) (-chassis.vx + chassis.vy +
                   chassis.wz *
                       (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) *
                       0.5) *
        CHASSIS_MOTOR_DEC_RATIO;

    /* may apply super super capacity gain here */
    /* may apply level up gain and power limit here when we have referee system feedback */
}

void ChassisApp::chassis_calc_wheel_pid_out() {
    mecanum_wheel_calc_speed();
    /* max +-16834 */
    for (int i = 0; i < 4; i++) {
        VAL_LIMIT(chassis.mec_spd[i], -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
        pid_single_loop_control(chassis.mec_spd[i], &(motor_controls[i].f_pid),
                                motor_controls[i].feedback.rx_rpm,
                                CHASSIS_TASK_EXEC_TIME * 0.001);
    }
}

void ChassisApp::send_swerve_angle_commands() {
    Motor_CAN_ID_t angle_can_ids[] = {
        SWERVE_STEER_MOTOR1,
        SWERVE_STEER_MOTOR2,
        SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4,
    };
    MotorSetMessage_t set_message;
    memset(&set_message, 0, sizeof(MotorSetMessage_t));

    for (int i = 0; i < 4; i++) {
        set_message.motor_can_volts[i] = ChassisApp::pack_lk_motor_message(
            true, chassis.swerve_spd[i], chassis.swerve_angle[i]);
        set_message.can_ids[i] = angle_can_ids[i];
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}

void ChassisApp::chassis_send_wheel_volts() {
    MotorSetMessage_t set_message;
    memset(&set_message, 0, sizeof(MotorSetMessage_t));

    for (int i = 0; i < 4; i++) {
        set_message.motor_can_volts[i] =
            (int32_t) motor_controls[i].f_pid.total_out;
        set_message.can_ids[i] = (Motor_CAN_ID_t) motor_controls[i].stdid;
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}

/*
 * @brief 	  Update chassis gimbal axis data through rc
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void ChassisApp::chassis_update_gimbal_coord(int16_t* channels) {
    /* controller data is not required to be filtered */
    chassis.gimbal_axis.vx = channels[2];  // apply vx data here
    chassis.gimbal_axis.vy = channels[3];  // apply vy data here
    chassis.gimbal_axis.wz = channels[0];  // apply wz data here
    //	else if(rc_hdlr->control_mode == PC_MODE){
    //		/* x axis process */
    //		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status == PRESSED)
    //			/* why did you do this bro ? */
    //			chassis_hdlr->gimbal_axis.vx = 0;
    //		/* both not pressed, brake slowly*/
    //		else if(rc_hdlr->pc.key.W.status != PRESSED && rc_hdlr->pc.key.S.status != PRESSED){
    //			chassis_brake(&chassis_hdlr->gimbal_axis.vx, 1.0f, 2.0f);
    //		}
    //
    //		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status != PRESSED){// check holding
    //			chassis_hdlr->gimbal_axis.vx += CHASSIS_PC_RAMP_VALUE; // apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->gimbal_axis.vx >= chassis_hdlr->max_vx)
    //				chassis_hdlr->gimbal_axis.vx = chassis_hdlr->max_vx;
    //		}
    //
    //		if(rc_hdlr->pc.key.S.status == PRESSED && rc_hdlr->pc.key.W.status != PRESSED){// check holding
    //			chassis_hdlr->gimbal_axis.vx -= CHASSIS_PC_RAMP_VALUE; // apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->gimbal_axis.vx < -chassis_hdlr->max_vx)
    //				chassis_hdlr->gimbal_axis.vx = -chassis_hdlr->max_vx;
    //		}
    //
    //		/* y axis process */
    //		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status == PRESSED)
    //			/* why did you do this bro ? */
    //			chassis_hdlr->gimbal_axis.vy = 0;
    //		/* both not pressed, brake slowly*/
    //		else if(rc_hdlr->pc.key.A.status != PRESSED && rc_hdlr->pc.key.D.status != PRESSED){
    //			chassis_brake(&chassis_hdlr->gimbal_axis.vy, 1.0f, 2.0f);
    //		}
    //
    //		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status != PRESSED){// check holding
    //			chassis_hdlr->gimbal_axis.vy -= CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->gimbal_axis.vy < -chassis_hdlr->max_vy)
    //				chassis_hdlr->gimbal_axis.vy = -chassis_hdlr->max_vy;
    //		}
    //
    //		if(rc_hdlr->pc.key.D.status == PRESSED && rc_hdlr->pc.key.A.status != PRESSED){// check holding
    //			chassis_hdlr->gimbal_axis.vy += CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->gimbal_axis.vy > chassis_hdlr->max_vy)
    //				chassis_hdlr->gimbal_axis.vy = chassis_hdlr->max_vy;
    //		}
    //
    //		/* angular velocity process, only used in Gimbal Follow mode */
    //		if(chassis_hdlr->chassis_act_mode == GIMBAL_FOLLOW){
    //			if(rc_hdlr->pc.key.Q.status == PRESSED && rc_hdlr->pc.key.E.status == PRESSED)
    //				/* why did you do this bro ? */
    //				chassis_hdlr->gimbal_axis.wz = 0;
    //			/* both not pressed, brake slowly*/
    //			else if(rc_hdlr->pc.key.Q.status != PRESSED && rc_hdlr->pc.key.E.status != PRESSED){
    //				chassis_brake(&chassis_hdlr->gimbal_axis.wz, 1.0f, 2.0f);
    //			}
    //
    //			if(rc_hdlr->pc.key.Q.status == PRESSED && rc_hdlr->pc.key.E.status != PRESSED){// check holding
    //				chassis_hdlr->gimbal_axis.wz -= CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
    //				if(chassis_hdlr->gimbal_axis.wz < -chassis_hdlr->max_wz)
    //					chassis_hdlr->gimbal_axis.wz = -chassis_hdlr->max_wz;
    //			}
    //
    //			if(rc_hdlr->pc.key.E.status == PRESSED && rc_hdlr->pc.key.Q.status != PRESSED){// check holding
    //				chassis_hdlr->gimbal_axis.wz += CHASSIS_PC_RAMP_VALUE;// apply ramp-like mode to engage chassis
    //				if(chassis_hdlr->gimbal_axis.wz > chassis_hdlr->max_vy)
    //					chassis_hdlr->gimbal_axis.wz = chassis_hdlr->max_vy;
    //			}
    //		}
    //	}
}
/*
 * @brief 	  Update chassis ground data through rc
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void ChassisApp::chassis_update_chassis_coord(int16_t* channels) {
    /*chassis coordinates only for debugging purpose, thus no pc control processing*/
    chassis.vx = channels[2];  // apply vx data here
    chassis.vy = channels[3];  // apply vy data here
    chassis.wz = channels[0];
    //	else if(rc_hdlr->control_mode == PC_MODE){
    //		/* x axis process */
    //		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status == PRESSED)
    //			/* why did you do this bro ? */
    //			chassis_hdlr->vx = 0;
    //		/* both not pressed, brake slowly*/
    //		else if(rc_hdlr->pc.key.W.status != PRESSED && rc_hdlr->pc.key.S.status != PRESSED){
    //			chassis_brake(&chassis_hdlr->vx, 1.0f, 2.0f);
    //		}
    //
    //		if(rc_hdlr->pc.key.W.status == PRESSED && rc_hdlr->pc.key.S.status != PRESSED){// check holding
    //			chassis_hdlr->vx += CHASSIS_PC_RAMP_VALUE ; // apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->vx >= chassis_hdlr->max_vx)
    //				chassis_hdlr->vx = chassis_hdlr->max_vx;
    //		}
    //
    //		if(rc_hdlr->pc.key.S.status == PRESSED && rc_hdlr->pc.key.W.status != PRESSED){// check holding
    //			chassis_hdlr->vx -= CHASSIS_PC_RAMP_VALUE ; // apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->vx < -chassis_hdlr->max_vx)
    //				chassis_hdlr->vx = -chassis_hdlr->max_vx;
    //		}
    //
    //		/* y axis process */
    //		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status == PRESSED)
    //			/* why did you do this bro ? */
    //			chassis_hdlr->vy = 0;
    //		/* both not pressed, brake slowly*/
    //		else if(rc_hdlr->pc.key.A.status != PRESSED && rc_hdlr->pc.key.D.status != PRESSED){
    //			chassis_brake(&chassis_hdlr->vy, 1.0f, 2.0f);
    //		}
    //
    //		if(rc_hdlr->pc.key.A.status == PRESSED && rc_hdlr->pc.key.D.status != PRESSED){// check holding
    //			chassis_hdlr->vy -= CHASSIS_PC_RAMP_VALUE ;// apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->vy < -chassis_hdlr->max_vy)
    //				chassis_hdlr->vy = -chassis_hdlr->max_vy;
    //		}
    //
    //		if(rc_hdlr->pc.key.D.status == PRESSED && rc_hdlr->pc.key.A.status != PRESSED){// check holding
    //			chassis_hdlr->vy += CHASSIS_PC_RAMP_VALUE ;// apply ramp-like mode to engage chassis
    //			if(chassis_hdlr->vy > chassis_hdlr->max_vy)
    //				chassis_hdlr->vy = chassis_hdlr->max_vy;
    //		}
    //
    //		chassis_hdlr->wz = 2.0*rc_hdlr->pc.mouse.x;
    //		if(chassis_hdlr->wz < -chassis_hdlr->max_wz)
    //			chassis_hdlr->wz = -chassis_hdlr->max_wz;
    ////		if(chassis_hdlr->wz > -chassis_hdlr->max_wz)
    ////			chassis_hdlr->wz = chassis_hdlr->max_wz;
    //
    //	}
}

/*
 * @brief Execute the chassis action mode:
 *			follow gimbal center| move_along gimbal coordinate |
 *			self-spinning while follow the gimbal coordinate | independent(ground coordinate)
 */
//FIXME: Didn't consider the acceleration. Acceleration can help us better explicit the buffer energy.
//		 But with more critical strict on power management.
void ChassisApp::chassis_exec_act_mode() {
    if (chassis.chassis_mode == IDLE_MODE) {
        chassis.vx = 0;
        chassis.vy = 0;
        chassis.wz = 0;
    } else if (chassis.chassis_act_mode == GIMBAL_CENTER) {  // gyro mode
        /* The front of chassis always chases gimbal yaw's ecd center (aka Twist mode) */
        chassis.vx = chassis.gimbal_axis.vx;
        chassis.vy = chassis.gimbal_axis.vy;
        chassis.wz = -pid_single_loop_control(0, &(chassis.f_pid),
                                              chassis.gimbal_yaw_rel_angle,
                                              CHASSIS_TASK_EXEC_TIME * 0.001);
    } else if (chassis.chassis_act_mode == GIMBAL_FOLLOW) {  // encoder mode
        /* The chassis always move along gimbal's coord/axis , but not chasing yaw's center */
        chassis.vx =
            chassis.gimbal_axis.vx * arm_cos_f32(chassis.gimbal_yaw_rel_angle) -
            chassis.gimbal_axis.vy * arm_sin_f32(chassis.gimbal_yaw_rel_angle);
        chassis.vy =
            chassis.gimbal_axis.vx * arm_sin_f32(chassis.gimbal_yaw_rel_angle) +
            chassis.gimbal_axis.vy * arm_cos_f32(chassis.gimbal_yaw_rel_angle);
        chassis.wz = 0;
        //		if(rc.control_mode == CTRLER_MODE)
        //			chassis_hdlr->wz = 0;
        //		else if(rc.control_mode == PC_MODE)
        //			chassis_hdlr->wz = chassis_hdlr->gimbal_axis.wz;
    } else if (chassis.chassis_act_mode == SELF_GYRO) {  // gyro or encoder mode
        /* The chassis always move along gimbal's coord/axis , meanwhile spinning the chassis with a fixed speed */
        chassis.vx =
            chassis.gimbal_axis.vx * arm_cos_f32(chassis.gimbal_yaw_rel_angle) -
            chassis.gimbal_axis.vy * arm_sin_f32(chassis.gimbal_yaw_rel_angle);
        chassis.vy =
            chassis.gimbal_axis.vx * arm_sin_f32(chassis.gimbal_yaw_rel_angle) +
            chassis.gimbal_axis.vy * arm_cos_f32(chassis.gimbal_yaw_rel_angle);
        /* for robots with slipring */
        //FIXME apply differential rotary control or use Q&E to change direction
        chassis.wz = CHASSIS_ECD_CONST_OMEGA * 3.5f;
    } else if (chassis.chassis_act_mode == INDPET_MODE) {  // encoder mode
        /* The chassis follow the ground axis
		 * Also can be used as sentry's chassis cmd
		 *  */
        chassis.vx = chassis.vx;
        chassis.vy = chassis.vy;
        chassis.wz = chassis.wz;  //CHASSIS_SLEF_GYRO_ANG_VEL * 1.0f;
    }

    /* set limit axis speed */
#ifdef CHASSIS_POWER_LIMIT
//	uint8_t cur_robot_level = referee.robot_status_data.robot_level;
//	/* Chassis Power Management Starts Here */
//	if(cur_robot_level > 10 || cur_robot_level < 1){
//		cur_robot_level = chassis_hdlr->prev_robot_level;// Set prev level for secure
//	}
//	else if(cur_robot_level - cur_robot_level >= 5){ // This may indicate failure comm with ref
//		cur_robot_level = chassis_hdlr->prev_robot_level;
//	}
//	select_chassis_speed(chassis_hdlr, cur_robot_level);
#endif
#ifndef REF_CHASSIS_DEBUG
    VAL_LIMIT(chassis.vx, -chassis.max_vx, chassis.max_vx);
    VAL_LIMIT(chassis.vy, -chassis.max_vy, chassis.max_vy);
    if (chassis.chassis_act_mode != GIMBAL_CENTER)
        // Gimbal center doesn't need to limit wz
        VAL_LIMIT(chassis.wz, -chassis.max_wz, chassis.max_wz);
    else
        VAL_LIMIT(chassis.wz, -1.5 * chassis.max_wz, 1.5 * chassis.max_wz);
#else
    VAL_LIMIT(chassis.vx, -temp_max_vx, temp_max_vx);
    VAL_LIMIT(chassis.vy, -temp_max_vy, temp_max_vy);
    VAL_LIMIT(chassis.wz, -temp_max_wz, temp_max_wz);
#endif
    if (fabs(chassis.wz) < 50.0f)
        /* PID dead zone risk management */
        chassis.wz = 0;
}

/*
 * @brief brake the chassis slowly to avoid instant power overlimt
 */
void ChassisApp::chassis_brake(float* vel, float ramp_step,
                               float stop_threshold) {
    if (*vel > 0)           // both release -> brake
        *vel -= ramp_step;  //brake need to be quicker
    else if (*vel < 0)
        *vel += ramp_step;
    if (fabs(*vel) < stop_threshold)
        *vel = 0;
}

void ChassisApp::chassis_get_gimbal_rel_angles() {
    float rel_angles[2];
    BaseType_t new_rel_angle_message =
        message_center.peek_message(GIMBAL_REL_ANGLES, rel_angles, 0);
    if (new_rel_angle_message == pdTRUE) {
        chassis.gimbal_yaw_rel_angle = rel_angles[0];
        chassis.gimbal_pitch_rel_angle = rel_angles[1];
    }
}

void ChassisApp::chassis_get_rc_info(int16_t* channels) {
    RCInfoMessage_t rc_info;
    BaseType_t new_message = message_center.peek_message(RC_INFO, &rc_info, 0);

    if (new_message == pdTRUE) {
        // TODO: Add input validation for modes and channels.
        BoardMode_t board_mode = static_cast<BoardMode_t>(rc_info.modes[0]);
        BoardActMode_t act_mode = static_cast<BoardActMode_t>(rc_info.modes[1]);

        chassis.chassis_mode = board_mode;
        chassis.chassis_act_mode = act_mode;
        memcpy(channels, &(rc_info.channels), sizeof(int16_t) * 4);
    }
}

void ChassisApp::chassis_get_wheel_feedback() {
    MotorReadMessage_t read_message;
    Motor_CAN_ID_t wheel_can_ids[] = {CHASSIS_WHEEL1, CHASSIS_WHEEL2,
                                      CHASSIS_WHEEL3, CHASSIS_WHEEL4};

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (int i = 0; i < 4; i++) {
            uint8_t good = 1;
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (wheel_can_ids[i] == read_message.can_ids[j]) {
                    memcpy(&(motor_controls[i].feedback),
                           &(read_message.feedback[j]),
                           sizeof(Motor_Feedback_t));
                    good = 0;
                    break;
                }
            }
        }
    }
}

/* define rc used count vars */
//int8_t chassis_pc_mode_toggle = 1; // encoder mode
//int8_t chassis_pc_submode_toggle = -1; // default to gimbal follow mode
//int32_t temp_toggle_count = 0;
/*
 * @brief     mode selection based on remote controller
 * @param[in] chassis: main chassis handler
 * @param[in] rc: main remote controller handler
 * */
//static void chassis_rc_mode_selection(Chassis_t* chassis_hdlr){

/* Mode quick check:
	 * -----------------------------------------------------------------------
	 * | toggle flag | pc_mode_toggle | pc_submode_toggle | Mode			 |
	 * |             | 		(Ctrl)    |        (F)        |      			 |
	 * -----------------------------------------------------------------------
	 * |  		     |  	 1  	  |  		-1		  |	GIMBAL_FOLLOW    | __
	 * |			 |--------------------------------------------------------   | - Enconder mode
	 * |	         |  	 1   	  |  	    1         |	INDEPENDENT MODE | __|
	 * |	index	 |--------------------------------------------------------
	 * |	         |      -1        |  		-1        |	GIMBAL CENTER    | __
	 * |			 |--------------------------------------------------------   | - Gyro mode
	 * |	         |  	-1        |  	    1         |	SELF_GYRO	     | __|
	 * -----------------------------------------------------------------------
	 * */
/* pc end mode selection */
//	else if(rc_hdlr->control_mode == PC_MODE) {
//		if(rc_hdlr->pc.key.key_buffer & KEY_BOARD_G){
//				/* if s1 down, then just shut down everything */
//				board_mode = IDLE_MODE;
//			}
//		else{
//			/* else just set up to patrol mode */
//			board_mode = PATROL_MODE;
//			/* update keys state */
////			if(rc_hdlr->pc.key.key_buffer & KEY_BOARD_CTRL)
//			if(rc_get_key_status(&rc_hdlr->pc.key.Ctrl) == RELEASED_TO_PRESS){ // check rising edge
//				chassis_pc_mode_toggle = -chassis_pc_mode_toggle;
//				temp_toggle_count++;
//			}
//			if(rc_get_key_status(&rc_hdlr->pc.key.F) == RELEASED_TO_PRESS) // check rising edge
//				chassis_pc_submode_toggle = -chassis_pc_submode_toggle;
//
//			/* mode decide */
//			if(chassis_pc_mode_toggle == -1 && chassis_pc_submode_toggle == -1){
//				/* chassis follow gimbal center while follow yaw axis */
//				act_mode = GIMBAL_CENTER;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//
//			else if(chassis_pc_mode_toggle == -1 && chassis_pc_submode_toggle == 1){
//				/* spinning chassis while follow yaw axis */
//				act_mode = SELF_GYRO;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//			else if(chassis_pc_mode_toggle == 1 && chassis_pc_submode_toggle == -1){
//				/* chassis only follow yaw axis */
//				act_mode = GIMBAL_FOLLOW;
//				/* update gimbal axis */
//				chassis_update_gimbal_coord(chassis_hdlr, rc_hdlr);
//			}
//			else if(chassis_pc_mode_toggle == 1 && chassis_pc_submode_toggle == 1){
//				/* independent mode */
//				act_mode = INDPET_MODE;
//				/* update ground axis */
//				chassis_update_chassis_coord(chassis_hdlr, rc_hdlr);
//			}
//		}// else patrol mode
//	}//pc mode

/* set modes */
//}

//void chassis_manual_gear_set(Chassis_t* chassis_hdlr, RemoteControl_t *rc_hdlr){
//	/* Manually set the levels of robot */
//	KeyStatus_t temp_upgrade_status = rc_get_key_status(&rc_hdlr->pc.key.V);
//	KeyStatus_t temp_downgrade_status = rc_get_key_status(&rc_hdlr->pc.key.Shift);
//	if( temp_upgrade_status == RELEASED_TO_PRESS && upgrade_pre_mode == RELEASED){
//		chassis_hdlr->cur_robot_level++;
//		upgrade_pre_mode = RELEASED_TO_PRESS;
//	}
//	else{
//		upgrade_pre_mode = temp_upgrade_status;
//	}
//	if(temp_downgrade_status == RELEASED_TO_PRESS && downgrade_pre_mode == RELEASED){
//		chassis_hdlr->cur_robot_level--;
//		downgrade_pre_mode = RELEASED_TO_PRESS;
//	}
//	else{
//		downgrade_pre_mode = temp_downgrade_status;
//	}
//
//	/* Safety check */
//	if(chassis_hdlr->cur_robot_level >= 10)
//		chassis_hdlr->cur_robot_level = 10;
//	if(chassis_hdlr->cur_robot_level <= 1)
//		chassis_hdlr->cur_robot_level = 1;
//
//	/* Set level */
//	switch(chassis_hdlr->cur_robot_level){
//			case 1: chassis_hdlr->max_vx = chassis_l1_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l1_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l1_hpf_spin_speed;break;
//			case 2: chassis_hdlr->max_vx = chassis_l2_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l2_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l2_hpf_spin_speed;break;
//			case 3: chassis_hdlr->max_vx = chassis_l3_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l3_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l3_hpf_spin_speed;break;
//			case 4: chassis_hdlr->max_vx = chassis_l4_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l4_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l4_hpf_spin_speed;break;
//			case 5: chassis_hdlr->max_vx = chassis_l5_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l5_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l5_hpf_spin_speed;break;
//			case 6: chassis_hdlr->max_vx = chassis_l6_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l6_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l6_hpf_spin_speed;break;
//			case 7: chassis_hdlr->max_vx = chassis_l7_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l7_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l7_hpf_spin_speed;break;
//			case 8: chassis_hdlr->max_vx = chassis_l8_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l8_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l8_hpf_spin_speed;break;
//			case 9: chassis_hdlr->max_vx = chassis_l9_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l9_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l9_hpf_spin_speed;break;
//			case 10: chassis_hdlr->max_vx = chassis_l10_hpf_padding_speed;
//					chassis_hdlr->max_vy = chassis_l10_hpf_padding_speed;
//					chassis_hdlr->max_wz = chassis_l10_hpf_spin_speed;break;
//	}
//}

void ChassisApp::select_chassis_speed(uint8_t level) {
    chassis.prev_robot_level = level;
    switch (level) {
        case 1:
            chassis.max_vx = chassis_l1_hpf_padding_speed;
            chassis.max_vy = chassis_l1_hpf_padding_speed;
            chassis.max_wz = chassis_l1_hpf_spin_speed;
            break;
        case 2:
            chassis.max_vx = chassis_l2_hpf_padding_speed;
            chassis.max_vy = chassis_l2_hpf_padding_speed;
            chassis.max_wz = chassis_l2_hpf_spin_speed;
            break;
        case 3:
            chassis.max_vx = chassis_l3_hpf_padding_speed;
            chassis.max_vy = chassis_l3_hpf_padding_speed;
            chassis.max_wz = chassis_l3_hpf_spin_speed;
            break;
        case 4:
            chassis.max_vx = chassis_l4_hpf_padding_speed;
            chassis.max_vy = chassis_l4_hpf_padding_speed;
            chassis.max_wz = chassis_l4_hpf_spin_speed;
            break;
        case 5:
            chassis.max_vx = chassis_l5_hpf_padding_speed;
            chassis.max_vy = chassis_l5_hpf_padding_speed;
            chassis.max_wz = chassis_l5_hpf_spin_speed;
            break;
        case 6:
            chassis.max_vx = chassis_l6_hpf_padding_speed;
            chassis.max_vy = chassis_l6_hpf_padding_speed;
            chassis.max_wz = chassis_l6_hpf_spin_speed;
            break;
        case 7:
            chassis.max_vx = chassis_l7_hpf_padding_speed;
            chassis.max_vy = chassis_l7_hpf_padding_speed;
            chassis.max_wz = chassis_l7_hpf_spin_speed;
            break;
        case 8:
            chassis.max_vx = chassis_l8_hpf_padding_speed;
            chassis.max_vy = chassis_l8_hpf_padding_speed;
            chassis.max_wz = chassis_l8_hpf_spin_speed;
            break;
        case 9:
            chassis.max_vx = chassis_l9_hpf_padding_speed;
            chassis.max_vy = chassis_l9_hpf_padding_speed;
            chassis.max_wz = chassis_l9_hpf_spin_speed;
            break;
        case 10:
            chassis.max_vx = chassis_l10_hpf_padding_speed;
            chassis.max_vy = chassis_l10_hpf_padding_speed;
            chassis.max_wz = chassis_l10_hpf_spin_speed;
            break;
    }
}

#ifdef CHASSIS_POWER_LIMIT
/******************************************************************************************************************
 * CHASSIS POWER MANAGEMENT PROCESS WALKTHROUGH
 ******************************************************************************************************************
 * 	@attention: rules:
 *  The chassis power consumption of robots will be continuously monitored by the Referee System, and the robot
	chassis needs to run within the chassis power consumption limit. Considering it is difficult for a robot to control
	instantaneous output power when in motion, a buffer energy (Z) has been defined to avoid the consequent penalty.
 *  The buffer energy value of Hero, Standard and Sentry Robots is 60J.
 *  Excess Percentage: K = (Pr－Pl) / Pl * 100%, where Pr is the instantaneous Chassis Power Consumption output and
	Pl is the power consumption limit.
 *	W = Pt, meaning that if we running infantry with power=100w (the limit of the power=40w, level 1 infantry), after 1s
 	Hp would be deducted.(t = (100-40)/buffer_energy)

 	Power limit:(RMNA 2023 Rules manual)							Buffer energy:60J
 				Mode			Power-focused		HP-focues
 	infantry  level 1				 60                 45
 			  level 2			     80					50
 			  level 3   			100					55

 Strategy:
 * 1) Decrease the current value of each Mecanum wheel in equal proportions so that the total power does not exceed
 * 	  the power limit
 ******************************************************************************************************************/

/*
 * @brief     the power management of chassis strategy 1: proportionally decrease based on referee callback.
 * @param[in] chassis: main chassis handler
 * */
void chassis_power_limit_referee(Chassis_t* chassis_hdlr) {
    int32_t total_current = 0;
    int32_t abs_current_weighted_sum = 0;
    //step 1： Read chassis power limit rx data from referee system (op: combine super capacity power buffer)
    get_chassis_ref_power_stat(chassis_hdlr, &referee);
    //step 2: Converts the power limit to the total maximum motor output current(threshold)

    //step 3: Compare the threshold with a DANGER value, determine the power limit ratio (possibly)
    /**************************************** Important FIXME **********************************************/
    //FIXME: We may need to consider using the current feedback to more precisely calc total current thus total
    //		 power(Pt = Sum(motor[i] for i=range(0:3)) * U(24v)), not simply hard-set - it may cause significant shift
    //		 if needed, we may also apply pid control for it.
    //FIXME: The condition here is too naive and not considering the sampling time
    //		 the critical condition should be like: W_buffer - W_danger = samplingTime * P_limit
    /**************************************** Important FIXME **********************************************/
    if (chassis_hdlr->ref_power_stat.power >=
        CHASSIS_POWER_THRESHOLD) {  // FIXME: here we need to change the threshold
        //based on the robots level feedback, refer to public defines
        // stop chassis immediately
        total_current = 0;
    }
    if (chassis_hdlr->ref_power_stat.power >= CHASSIS_POWER_DANGER) {

        // decrease the chassis spd immediately
        total_current = CHASSIS_POWER_DANGER *
                        POWER_TO_CURRENT;  //random value, need to test
    }
    //step 4: Weighted assignment (soft-max filter) of the current motor output value (conditional judgment)
    //softmax(chassis_hdlr->mec_spd, 4); //softmax cannot handle negative value very well
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        chassis_hdlr->mec_spd[i] =
            chassis_hdlr->mec_spd[i] *
            (abs(chassis_hdlr->mec_spd[i]) / abs_current_weighted_sum);
}

/*
 * @brief     the power management of chassis strategy 2: proportionally decrease based on local motor data.
 * @param[in] chassis: main chassis handler
 * @param[in] local_power_limit
 *
 * @note: this function calculates the power based on the equation below:
 * 			P = UI, where the voltage U is 24V, and the current can be obtained from can feedbcak data.
 * */
void chassis_power_limit_local(Chassis_t* chassis_hdlr,
                               uint16_t local_power_limit) {
    /* step 1: get the current power */
    int16_t current_power = 0;
    int32_t total_current = 0;
    int32_t abs_current_weighted_sum = 0;

    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
        current_power +=
            24 * motor_data[i].motor_feedback.rx_current * CHASSIS_MAX_SPEED /
            20;  //assume the feedbcak current has same range with tx data
    }

    /* check if the power over danger zone */
    if (current_power >=
        CHASSIS_POWER_THRESHOLD) {  //assume we don't have a power feedbcak, need to manually set limit
        total_current = 0;
    } else if (current_power >= local_power_limit) {
        total_current = (int32_t) local_power_limit / 24;
    }

    /*apply current */
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        abs_current_weighted_sum += abs(chassis_hdlr->mec_spd[i]);
    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++)
        chassis_hdlr->mec_spd[i] =
            (int16_t) chassis_hdlr->mec_spd[i] *
            (abs(chassis_hdlr->mec_spd[i]) / abs_current_weighted_sum);
}

/* get the latest current power and power limit from referee system */
void get_chassis_ref_power_stat(Chassis_t* chassis_hdlr, Referee_t* ref) {
    chassis_hdlr->ref_power_stat.current = ref->power_heat_data.chassis_current;
    chassis_hdlr->ref_power_stat.power = ref->power_heat_data.chassis_power;
    chassis_hdlr->ref_power_stat.buffer_energy =
        ref->power_heat_data.buffer_energy;
}
#endif

/* only for sentry begin */

#endif /*__CHASSIS_APP__*/
