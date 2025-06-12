#include "Swerve_Drive.h"
#include <algorithm>
#include <cstring>
#include <iterator>
#include <numbers>
#include "apps_defines.h"
#include "motors.h"
#include "pid.h"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"

SwerveDrive::SwerveDrive(IMessageCenter& message_center_ref,
                         IMotors& motors_ref, float width_, float dt_)
    : message_center(message_center_ref),
      motors(motors_ref),
      width(width_),
      dt(dt_) {}

void SwerveDrive::init_impl() {
    steer_curr_angle = {0};
    steer_curr_speed = {0};
    steer_cw_mag = {0};
    steer_ccw_mag = {0};
    steer_target_angle = {0};
    drive_target_speed = {0};

    steer_max_speed = {0};
    steer_ccw = {false};
    steer_output_angle = {0};
    drive_output = {0};

    for (size_t i = 0; i < drive_motors.size(); i++) {
        drive_motors.at(i).stdid = 0;
        pid2_init(drive_motors.at(i).f_pid,
                  robot_config::pid_params::KP_SWERVE_DRIVE,
                  robot_config::pid_params::KI_SWERVE_DRIVE,
                  robot_config::pid_params::KD_SWERVE_DRIVE,
                  robot_config::pid_params::BETA_SWERVE_DRIVE,
                  robot_config::pid_params::YETA_SWERVE_DRIVE,
                  robot_config::pid_params::MIN_OUT_SWERVE_DRIVE,
                  robot_config::pid_params::MAX_OUT_SWERVE_DRIVE);
        drive_motors.at(i).feedback.rx_angle = 0;
        drive_motors.at(i).feedback.rx_current = 0;
        drive_motors.at(i).feedback.rx_rpm = 0;
        drive_motors.at(i).feedback.rx_temp = 0;
    }

    for (size_t i = 0; i < steer_motors.size(); i++) {
        steer_motors.at(i).stdid = 0;
        steer_motors.at(i).lk_feedback.ecd_position = 0;
        steer_motors.at(i).lk_feedback.speed = 0;
        steer_motors.at(i).lk_feedback.torque_current = 0;
        steer_motors.at(i).lk_feedback.temperature = 0;
    }

    std::get<0>(drive_motors).stdid = CHASSIS_WHEEL1;
    std::get<1>(drive_motors).stdid = CHASSIS_WHEEL2;
    std::get<2>(drive_motors).stdid = CHASSIS_WHEEL3;
    std::get<3>(drive_motors).stdid = CHASSIS_WHEEL4;

    std::get<0>(steer_motors).stdid = SWERVE_STEER_MOTOR1;
    std::get<1>(steer_motors).stdid = SWERVE_STEER_MOTOR2;
    std::get<2>(steer_motors).stdid = SWERVE_STEER_MOTOR3;
    std::get<3>(steer_motors).stdid = SWERVE_STEER_MOTOR4;
}

void SwerveDrive::get_motor_feedback() {
    MotorReadMessage_t read_message;

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (size_t i = 0; i < MAX_MOTOR_COUNT; i++) {
            for (size_t j = 0; j < steer_motors.size(); j++) {
                if (steer_motors[j].stdid == read_message.can_ids[i]) {
                    motors.get_raw_feedback(steer_motors.at(j).stdid,
                                            read_message.feedback[i],
                                            &(steer_motors.at(j).lk_feedback));
                    break;
                }
            }

            for (size_t j = 0; j < drive_motors.size(); j++) {
                if (drive_motors.at(j).stdid == read_message.can_ids[i]) {
                    motors.get_raw_feedback(drive_motors.at(j).stdid,
                                            read_message.feedback[i],
                                            &(drive_motors.at(j).feedback));
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
     */
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
    float zero3 = 340;
    float zero4 = 195.5;

    /* Realign domain with relative zero of MG4005 (we now multiply by 10 to convert to dps) */
    steer_target_angle.at(0) = realign(theta1, zero1);
    steer_target_angle.at(1) = realign(theta2, zero2);
    steer_target_angle.at(2) = realign(theta3, zero3);
    steer_target_angle.at(3) = realign(theta4, zero4);

    /* Set max rotaional speed (Ω) of MG4005 in dps */
    constexpr uint16_t STEER_MAX_SPEED = 4500;
    constexpr float ECD_SCALE_FACTOR = 360.0f / 65535.0f;
    constexpr float ANGLE_THRESHOLD = 5.0f;
    constexpr int16_t SPEED_THRESHOLD = 0;
    for (size_t i = 0; i < NUM_STEER_MOTORS; i++) {
        steer_max_speed.at(i) = STEER_MAX_SPEED;

        steer_curr_angle.at(i) =
            ECD_SCALE_FACTOR *
            static_cast<float>(steer_motors.at(i).lk_feedback.ecd_position);

        steer_curr_speed.at(i) = steer_motors.at(i).lk_feedback.speed;
        steer_cw_mag.at(i) =
            relative_angle(steer_curr_angle.at(i), steer_target_angle.at(i));
        steer_ccw_mag.at(i) =
            relative_angle(steer_target_angle.at(i), steer_curr_angle.at(i));

        if (std::min(steer_ccw_mag.at(i), steer_cw_mag.at(i)) >=
                ANGLE_THRESHOLD &&
            abs(steer_curr_speed.at(i)) <= SPEED_THRESHOLD) {
            steer_output_angle.at(i) = steer_target_angle.at(i);
            steer_ccw.at(i) = steer_ccw_mag.at(i) < steer_cw_mag.at(i);
        }
    }

    /* For speed (Vi) control of M3508 */
    std::get<0>(drive_target_speed) = sqrt(pow(B, 2) + pow(D, 2));
    std::get<1>(drive_target_speed) = sqrt(pow(B, 2) + pow(C, 2));
    std::get<2>(drive_target_speed) = sqrt(pow(A, 2) + pow(C, 2));
    std::get<3>(drive_target_speed) = sqrt(pow(A, 2) + pow(D, 2));

    constexpr float inverse_wheel_radius = 1 / 0.0508;
    constexpr float RADS_TO_RPM = 60 / (2 * PI);
    for (size_t i = 0; i < NUM_DRIVE_MOTORS; i++) {
        pid2_single_loop_control(
            drive_motors.at(i).f_pid,
            drive_target_speed.at(i) * RADS_TO_RPM * CHASSIS_MOTOR_DEC_RATIO *
                inverse_wheel_radius,
            static_cast<float>(drive_motors.at(i).feedback.rx_rpm), dt);
        drive_output.at(i) =
            static_cast<int32_t>(drive_motors.at(i).f_pid.total_out);
    }
}

void SwerveDrive::send_motor_messages() {
    static_assert(NUM_STEER_MOTORS + NUM_DRIVE_MOTORS <= MAX_MOTOR_COUNT);
    MotorSetMessage_t set_message;
    std::memset(&set_message, 0, sizeof(MotorSetMessage_t));
    for (size_t i = 0; i < NUM_STEER_MOTORS; i++) {
        set_message.motor_can_volts[i] = SwerveDrive::pack_lk_motor_message(
            steer_ccw.at(i), steer_max_speed.at(i),
            steer_output_angle.at(i) * 100);
        set_message.can_ids[i] =
            static_cast<Motor_CAN_ID_t>(steer_motors.at(i).stdid);
    }

    for (size_t i = 0; i < NUM_DRIVE_MOTORS; i++) {
        set_message.motor_can_volts[NUM_STEER_MOTORS + i] = drive_output.at(i);
        set_message.can_ids[NUM_STEER_MOTORS + i] =
            static_cast<Motor_CAN_ID_t>(drive_motors.at(i).stdid);
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}

int32_t SwerveDrive::pack_lk_motor_message(bool spin_ccw, uint16_t max_speed,
                                           uint32_t angle) {
    // ASSERT(angle <= 36000, "LK angle cannot be greater than 36000");
    ASSERT(max_speed <= 16383,
           "Max speeds higher than 16383 are not supported");
    int32_t motor_message_value = 0;
    motor_message_value |= (((uint32_t) max_speed) & 0x3fff) << 16;
    motor_message_value |= angle & 0xffff;
    motor_message_value |= (uint32_t) spin_ccw << 30;

    return motor_message_value;
}

float SwerveDrive::calc_power_consumption() {
    // TODO: Implement.
    return 0.f;
}