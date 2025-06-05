#include "Swerve_Drive.h"
#include <cstring>
#include <numbers>
#include "message_center.h"
#include "motors.h"
#include "pid.h"
#include "public_defines.h"
#include "uarm_lib.h"
#include "uarm_math.h"

SwerveDrive::SwerveDrive(IMessageCenter& message_center_ref,
                         float chassis_width)
    : message_center(message_center_ref), width(chassis_width) {}

void SwerveDrive::init_impl() {
    std::memset(steer_target_angle, 0, sizeof(float) * 4);
    std::memset(steer_max_speed, 0, sizeof(uint16_t) * 4);
    std::memset(steer_ccw, 0, sizeof(bool) * 4);
    std::memset(drive_target_rpm, 0, sizeof(int16_t) * 4);

    for (size_t i = 0; i < swerve_motors.size(); i++) {
        std::memset(&(swerve_motors[i].feedback), 0, sizeof(Motor_Feedback_t));
        swerve_motors[i].angle = 0;
    }

    swerve_motors[DRIVE_MOTOR1_INDEX].stdid = CHASSIS_WHEEL1;
    swerve_motors[DRIVE_MOTOR2_INDEX].stdid = CHASSIS_WHEEL2;
    swerve_motors[DRIVE_MOTOR3_INDEX].stdid = CHASSIS_WHEEL3;
    swerve_motors[DRIVE_MOTOR4_INDEX].stdid = CHASSIS_WHEEL4;
    swerve_motors[STEER_MOTOR1_INDEX].stdid = SWERVE_STEER_MOTOR1;
    swerve_motors[STEER_MOTOR2_INDEX].stdid = SWERVE_STEER_MOTOR2;
    swerve_motors[STEER_MOTOR3_INDEX].stdid = SWERVE_STEER_MOTOR3;
    swerve_motors[STEER_MOTOR4_INDEX].stdid = SWERVE_STEER_MOTOR4;
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
    /* Define variables */
    float A = vx - wz * (width * 0.5);
    float B = vx + wz * (width * 0.5);
    float C = vy - wz * (width * 0.5);
    float D = vy + wz * (width * 0.5);
    float theta1 = atan2(B, D) * 180 / PI;
    float theta2 = atan2(B, C) * 180 / PI;
    float theta3 = atan2(A, C) * 180 / PI;
    float theta4 = atan2(A, D) * 180 / PI;

    /* Zero position of MG4005 (only needed if motor zero are not pointing in the "forward" direction) */
    float zero1 = 330;
    float zero2 = 166.5;
    float zero3 = 312.5;
    float zero4 = 195.5;

    /* Realign domain with relative zero of MG4005 (we now multiply by 10 to convert to dps) */
    float pos1 = realign(theta1, zero1) * 10;
    float pos2 = realign(theta2, zero2) * 10;
    float pos3 = realign(theta3, zero3) * 10;
    float pos4 = realign(theta4, zero4) * 10;

    /* For speed (Vi) control of M3508 */
    drive_target_rpm[OUTPUT1_INDEX] =
        static_cast<int16_t>(sqrt(pow(B, 2) + pow(D, 2)));
    drive_target_rpm[OUTPUT2_INDEX] =
        static_cast<int16_t>(sqrt(pow(B, 2) + pow(C, 2)));
    drive_target_rpm[OUTPUT3_INDEX] =
        static_cast<int16_t>(sqrt(pow(A, 2) + pow(C, 2)));
    drive_target_rpm[OUTPUT4_INDEX] =
        static_cast<int16_t>(sqrt(pow(A, 2) + pow(D, 2)));

    /* Set max rotaional speed (Ω) of MG4005 in dps */
    steer_max_speed[OUTPUT1_INDEX] = static_cast<uint16_t>(9000);
    steer_max_speed[OUTPUT2_INDEX] = static_cast<uint16_t>(9000);
    steer_max_speed[OUTPUT3_INDEX] = static_cast<uint16_t>(9000);
    steer_max_speed[OUTPUT4_INDEX] = static_cast<uint16_t>(9000);

    /* For angle (0i) control of MG4005 */
    steer_ccw[OUTPUT1_INDEX] =
        swerve_motors[STEER_MOTOR1_INDEX].angle - pos1 < 0;
    steer_ccw[OUTPUT2_INDEX] =
        swerve_motors[STEER_MOTOR2_INDEX].angle - pos2 < 0;
    steer_ccw[OUTPUT3_INDEX] =
        swerve_motors[STEER_MOTOR3_INDEX].angle - pos3 < 0;
    steer_ccw[OUTPUT4_INDEX] =
        swerve_motors[STEER_MOTOR4_INDEX].angle - pos4 < 0;

    steer_target_angle[OUTPUT1_INDEX] = pos1;
    steer_target_angle[OUTPUT2_INDEX] = pos2;
    steer_target_angle[OUTPUT3_INDEX] = pos3;
    steer_target_angle[OUTPUT4_INDEX] = pos4;
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
            steer_ccw[i], steer_max_speed[i], steer_target_angle[i]);
        set_message.can_ids[i] = angle_can_ids[i];
    }

    // TODO: Add chassis wheel speeds to set_message.

    message_center.pub_message(MOTOR_SET, &set_message);
}

int32_t SwerveDrive::pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                           uint32_t angle) {
    ASSERT(angle <= 36000, "LK angle cannot be greater than 36000");
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