#include "Swerve_Drive.h"
#include <cstring>
#include "motors.h"
#include "uarm_lib.h"

SwerveDrive::SwerveDrive(IMessageCenter& message_center_ref,
                         uint32_t chassis_width)
    : message_center(message_center_ref), width(chassis_width) {}

void SwerveDrive::init_impl() {
    std::memset(target_wheel_angles, 0, sizeof(float) * 4);
    std::memset(target_wheel_speeds, 0, sizeof(int16_t) * 4);

    for (size_t i = 0; i < swerve_motors.size(); i++) {
        std::memset(&(swerve_motors[i].feedback), 0, sizeof(Motor_Feedback_t));
        swerve_motors[i].angle = 0;
    }

    swerve_motors[0].stdid = CHASSIS_WHEEL1;
    swerve_motors[1].stdid = CHASSIS_WHEEL2;
    swerve_motors[2].stdid = CHASSIS_WHEEL3;
    swerve_motors[3].stdid = CHASSIS_WHEEL4;
    swerve_motors[4].stdid = SWERVE_STEER_MOTOR1;
    swerve_motors[5].stdid = SWERVE_STEER_MOTOR2;
    swerve_motors[6].stdid = SWERVE_STEER_MOTOR3;
    swerve_motors[7].stdid = SWERVE_STEER_MOTOR4;
    // TODO: init PID_t structs for drive wheels.
}

void SwerveDrive::get_motor_feedback() {
    MotorReadMessage_t read_message;

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (size_t i = 0; i < swerve_motors.size(); i++) {
            for (size_t j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (swerve_motors[i].stdid == read_message.can_ids[j]) {
                    switch (Motors::get_motor_brand(swerve_motors[i].stdid)) {
                        case LK:
                            Motors::parse_feedback(swerve_motors[i].stdid,
                                                   read_message.feedback[i],
                                                   &(swerve_motors[i].angle));
                            break;
                        case DJI:
                            Motors::parse_feedback(
                                swerve_motors[i].stdid,
                                read_message.feedback[i],
                                &(swerve_motors[i].feedback));
                            break;
                        default:
                            continue;
                    }
                    break;
                }
            }
        }
    }
}

/*
 * @brief 	  Inversely kinematics of swerve
 * @param[in] chassis_hdlr:chassis main struct
 * @retval    None
 */
void SwerveDrive::calc_motor_outputs(float vx, float vy, float wz) {
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

void SwerveDrive::send_motor_messages() {
    Motor_CAN_ID_t angle_can_ids[] = {
        SWERVE_STEER_MOTOR1, SWERVE_STEER_MOTOR2, SWERVE_STEER_MOTOR3,
        SWERVE_STEER_MOTOR4, CHASSIS_WHEEL1,      CHASSIS_WHEEL2,
        CHASSIS_WHEEL3,      CHASSIS_WHEEL4,
    };
    MotorSetMessage_t set_message;
    memset(&set_message, 0, sizeof(MotorSetMessage_t));

    for (int i = 0; i < 4; i++) {
        set_message.motor_can_volts[i] = SwerveDrive::pack_lk_motor_message(
            true, target_wheel_speeds[i], target_wheel_angles[i]);
        set_message.can_ids[i] = angle_can_ids[i];
    }

    // TODO: Add chassis wheel speeds to set_message.

    message_center.pub_message(MOTOR_SET, &set_message);
}

int32_t SwerveDrive::pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                           uint32_t angle) {
    ASSERT(angle < 40000, "LK angle cannot be greater than 36000");
    int32_t motor_message_value = 0;
    motor_message_value |= (((uint32_t) max_speed) & 0x0fff) << 16;
    motor_message_value |= angle & 0xffff;
    motor_message_value |= (int32_t) spin_ccw << 31;

    return motor_message_value;
}

float SwerveDrive::calc_power_consumption() {
    // TODO: Implement.
    return 0.f;
}