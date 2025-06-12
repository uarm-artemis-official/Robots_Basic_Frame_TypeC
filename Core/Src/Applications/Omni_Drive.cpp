#include "Omni_Drive.h"
#include <cstring>
#include "pid.h"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"

OmniDrive::OmniDrive(IMessageCenter& message_center_ref, IMotors& motors_ref,
                     float chassis_width, float chassis_length)
    : message_center(message_center_ref),
      motors(motors_ref),
      width(chassis_width),
      length(chassis_length) {}

void OmniDrive::init_impl() {
    memset(motor_angluar_vel, 0, sizeof(int16_t) * 4);

    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
        std::memset(&(motor_controls[i]), 0, sizeof(Chassis_Wheel_Control_t));
        pid2_init(motor_controls[i].f_pid,
                  robot_config::pid_params::KP_OMNI_DRIVE,
                  robot_config::pid_params::KI_OMNI_DRIVE,
                  robot_config::pid_params::KD_OMNI_DRIVE,
                  robot_config::pid_params::BETA_OMNI_DRIVE,
                  robot_config::pid_params::YETA_OMNI_DRIVE,
                  robot_config::pid_params::MIN_OUT_OMNI_DRIVE,
                  robot_config::pid_params::MAX_OUT_OMNI_DRIVE);
    }

    motor_controls[0].stdid = CHASSIS_WHEEL1;
    motor_controls[1].stdid = CHASSIS_WHEEL2;
    motor_controls[2].stdid = CHASSIS_WHEEL3;
    motor_controls[3].stdid = CHASSIS_WHEEL4;
}

void OmniDrive::get_motor_feedback() {
    MotorReadMessage_t read_message;
    Motor_CAN_ID_t wheel_can_ids[] = {CHASSIS_WHEEL1, CHASSIS_WHEEL2,
                                      CHASSIS_WHEEL3, CHASSIS_WHEEL4};

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (wheel_can_ids[i] == read_message.can_ids[j]) {
                    motors.get_raw_feedback(wheel_can_ids[i],
                                            read_message.feedback[j],
                                            &(motor_controls[i].feedback));
                    break;
                }
            }
        }
    }
}

void OmniDrive::calc_target_motor_speeds(float vx, float vy, float wz) {
    /* vx - chassis-relative horizontal velocity (m/s)
     * vy - chassis-relative vertical velocity (m/s)
     * wz - chassis-relative rotation (rad/s)
     * 
     * Velocity sign notation:
     *     +
     *   - * +
     *     -
    */
    /* Assume we install the mecanum wheels as O type (also have X type), right hand define positive dir
	 *			 x length
	 *		 v1  \\ -- //  v2     <Front>		   	 A      __			wheels define:
	 *		      |    |		y length			 | vy  /		 	 		  ___
	 *		  	  |	   |                             |     \__>   wz    \\ ->    | \ |
	 *		 v4	 // -- \\  v3     <Rear>             -----> vx  				 | \ |
	 *																			 |___|
	 *		--	vector([vx, vy, wz]) --
     *
     * See the below website about mecanum wheel kinematics:
     * https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html 
	 * */
    /* X type installation */

    /* may apply super super capacity gain here */
    /* may apply level up gain and power limit here when we have referee system feedback */
    constexpr float inverse_wheel_radius = 1 / CHASSIS_OMNI_WHEEL_RADIUS;
    motor_angluar_vel[0] = static_cast<int16_t>(
        (vx + vy + wz * (width + length) * 0.5) * inverse_wheel_radius);
    motor_angluar_vel
        [1] = /* We will put a negative infront of the eq. as motor install is flipped*/
        static_cast<int16_t>(
            -((-vx + vy - wz * (width + length) * 0.5) * inverse_wheel_radius));
    motor_angluar_vel
        [2] = /* We will put a negative infront of the eq. as motor install is flipped*/
        static_cast<int16_t>(
            -((vx + vy - wz * (width + length) * 0.5) * inverse_wheel_radius));
    motor_angluar_vel[3] = static_cast<int16_t>(
        (-vx + vy + wz * (width + length) * 0.5) * inverse_wheel_radius);
}

void OmniDrive::calc_motor_volts() {
    constexpr float RADS_TO_RPM = 60 / (2 * PI);
    for (int i = 0; i < 4; i++) {
        VAL_LIMIT(motor_angluar_vel[i], -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
        pid2_single_loop_control(
            motor_controls[i].f_pid,
            static_cast<float>(motor_angluar_vel[i] * RADS_TO_RPM *
                               CHASSIS_MOTOR_DEC_RATIO),
            static_cast<float>(motor_controls[i].feedback.rx_rpm),
            CHASSIS_TASK_EXEC_TIME * 0.001);
    }
}

void OmniDrive::calc_motor_outputs(float vx, float vy, float wz) {
    calc_target_motor_speeds(vx, vy, wz);
    calc_motor_volts();
}

void OmniDrive::send_motor_messages() {
    MotorSetMessage_t set_message;
    memset(&set_message, 0, sizeof(MotorSetMessage_t));

    for (int i = 0; i < 4; i++) {
        set_message.motor_can_volts[i] =
            (int32_t) motor_controls[i].f_pid.total_out;
        set_message.can_ids[i] = (Motor_CAN_ID_t) motor_controls[i].stdid;
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}

float OmniDrive::calc_power_consumption() {
    // TODO:: Implement.
    return 0.f;
}