#include "Omni_Drive.h"
#include <cstring>
#include <numeric>
#include "pid.h"
#include "ramp.hpp"
#include "robot_config.hpp"
#include "uarm_lib.h"
#include "uarm_math.h"

OmniDrive::OmniDrive(IMessageCenter& message_center_ref, IMotors& motors_ref,
                     float chassis_width, float chassis_length,
                     float power_limit_, float chassis_dt_)
    : message_center(message_center_ref),
      motors(motors_ref),
      width(chassis_width),
      length(chassis_length),
      power_limit(power_limit_),
      chassis_dt(chassis_dt_) {}

void OmniDrive::init_impl() {
    for (size_t i = 0; i < motor_controls.size(); i++) {
        motor_angular_vel.at(i) = 0;
        std::memset(&(motor_controls.at(i)), 0,
                    sizeof(Chassis_Wheel_Control_t));
        pid2_init(motor_controls.at(i).f_pid,
                  robot_config::chassis_params::KP_OMNI_DRIVE,
                  robot_config::chassis_params::KI_OMNI_DRIVE,
                  robot_config::chassis_params::KD_OMNI_DRIVE,
                  robot_config::chassis_params::BETA_OMNI_DRIVE,
                  robot_config::chassis_params::YETA_OMNI_DRIVE,
                  robot_config::chassis_params::MIN_OUT_OMNI_DRIVE,
                  robot_config::chassis_params::MAX_OUT_OMNI_DRIVE);
        ramp_init(motor_controls.at(i).sp_ramp,
                  robot_config::chassis_params::WHEEL_RAMP_MAX_ACCEL);
    }

    std::get<0>(motor_controls).stdid = CHASSIS_WHEEL1;
    std::get<1>(motor_controls).stdid = CHASSIS_WHEEL2;
    std::get<2>(motor_controls).stdid = CHASSIS_WHEEL3;
    std::get<3>(motor_controls).stdid = CHASSIS_WHEEL4;
}

void OmniDrive::get_motor_feedback() {
    MotorReadMessage_t read_message;
    Motor_CAN_ID_t wheel_can_ids[] = {CHASSIS_WHEEL1, CHASSIS_WHEEL2,
                                      CHASSIS_WHEEL3, CHASSIS_WHEEL4};

    uint8_t new_read_message =
        message_center.peek_message(MOTOR_READ, &read_message, 0);
    if (new_read_message == 1) {
        for (size_t i = 0; i < motor_controls.size(); i++) {
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (wheel_can_ids[i] == read_message.can_ids[j]) {
                    motors.get_raw_feedback(wheel_can_ids[i],
                                            read_message.feedback[j],
                                            &(motor_controls.at(i).feedback));
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
    std::get<0>(motor_angular_vel) =
        (vx + vy + wz * (width + length) * 0.5) * inverse_wheel_radius;
    std::get<1>(
        motor_angular_vel) = /* We will put a negative infront of the eq. as motor install is flipped*/
        -((-vx + vy - wz * (width + length) * 0.5) * inverse_wheel_radius);
    std::get<2>(
        motor_angular_vel) = /* We will put a negative infront of the eq. as motor install is flipped*/
        -((vx + vy - wz * (width + length) * 0.5) * inverse_wheel_radius);
    std::get<3>(motor_angular_vel) =
        (-vx + vy + wz * (width + length) * 0.5) * inverse_wheel_radius;
}

void OmniDrive::calc_wheel_power_consumption() {
    constexpr float CURRENT_RESOLUTION = 20.f / 16384.f;
    constexpr float OUTPUT_TORQUE_CONSTANT = 0.3 * 187 / 3591;
    for (size_t i = 0; i < motor_controls.size(); i++) {
        float angular_speed = motor_controls.at(i).feedback.rx_rpm == 0
                                  ? 0.01
                                  : motor_controls.at(i).feedback.rx_rpm;
        float torque = motor_controls.at(i).feedback.rx_current *
                       CURRENT_RESOLUTION * OUTPUT_TORQUE_CONSTANT;
        wheel_power_consumption.at(i) = torque * angular_speed / 9.55 +
                                        k1 * angular_speed * angular_speed +
                                        k2 * torque * torque + a;
    }
}

void OmniDrive::calc_power_limits() {
    calc_wheel_power_consumption();

    float vel_sum = 0;
    for (size_t i = 0; i < motor_angular_vel.size(); i++) {
        vel_sum += fabs(motor_angular_vel.at(i));
    }

    constexpr float CURRENT_RESOLUTION = 20.f / 16384.f;
    constexpr float OUTPUT_TORQUE_CONSTANT = 0.3 * 187 / 3591;

    for (size_t i = 0; i < motor_controls.size(); i++) {
        float allocated_motor_power;
        if (vel_sum == 0) {
            allocated_motor_power = power_limit / 4;
        } else {
            allocated_motor_power =
                fabs(motor_angular_vel.at(i)) / vel_sum * power_limit;
        }
        float angular_speed = motor_controls.at(i).feedback.rx_rpm == 0
                                  ? 0.01
                                  : motor_controls.at(i).feedback.rx_rpm;
        float torque = allocated_motor_power * 9.55 / angular_speed;
        float torque_current = torque / OUTPUT_TORQUE_CONSTANT;
        float output_limit =
            fabs(value_limit(torque_current / CURRENT_RESOLUTION,
                             -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED));
        pid2_set_limits(motor_controls.at(i).f_pid, -output_limit,
                        output_limit);
    }
}

void OmniDrive::calc_motor_volts() {
    calc_power_limits();

    constexpr float RADS_TO_RPM = 60 / (2 * PI);
    for (size_t i = 0; i < motor_controls.size(); i++) {
        int16_t motor_target = value_limit(
            motor_angular_vel.at(i), -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
        ramp_set_target(motor_controls.at(i).sp_ramp,
                        motor_controls.at(i).feedback.rx_rpm / RADS_TO_RPM /
                            CHASSIS_MOTOR_DEC_RATIO,
                        motor_target);

        ramp_calc_output(motor_controls.at(i).sp_ramp, chassis_dt);

        pid2_single_loop_control(
            motor_controls.at(i).f_pid,
            motor_controls.at(i).sp_ramp.output * RADS_TO_RPM *
                CHASSIS_MOTOR_DEC_RATIO,
            static_cast<float>(motor_controls.at(i).feedback.rx_rpm),
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
        set_message.motor_can_volts[i] = static_cast<int32_t>(
            std::roundf(motor_controls.at(i).f_pid.total_out));
        set_message.can_ids[i] =
            static_cast<Motor_CAN_ID_t>(motor_controls.at(i).stdid);
    }

    message_center.pub_message(MOTOR_SET, &set_message);
}

// Assumes that wheels are always running at max output and will be limited
// by max_out setting in pid.
float OmniDrive::calc_power_consumption() {
    float current_draw = 0;
    for (size_t i = 0; i < motor_controls.size(); i++) {
        current_draw += (motor_controls.at(i).f_pid.total_out) / 16834.f * 20.f;
    }
    return current_draw * 24;
}

void OmniDrive::set_max_power_impl(float new_max_power) {
    power_limit = new_max_power;
}