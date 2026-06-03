#include <cstring>
#include <numeric>
#include "apps_classes.hpp"
#include "apps_types.hpp"
#include "pid.h"
#include "ramp.hpp"
#include "robot_config.hpp"
#include "uarm_lib.hpp"
#include "uarm_math.hpp"

OmniDrive::OmniDrive(const apps::chassis::OmniDriveConfig& config_ref,
                     mc2::RobotMC& mc_ref, IMotors& motors_ref)
    : config(config_ref), mc(mc_ref), motors(motors_ref) {}

bool OmniDrive::init_impl() {
    for (size_t i = 0; i < motor_controls.size(); i++) {
        motor_angular_vel.at(i) = 0;
        std::memset(&(motor_controls.at(i)), 0,
                    sizeof(Chassis_Wheel_Control_t));
        pid2_init(motor_controls.at(i).f_pid, config.wheel_pid_config);
        ramp_init(motor_controls.at(i).sp_ramp,
                  config.max_wheel_ramp_accel.get());
    }

    std::get<0>(motor_controls).stdid = CHASSIS_WHEEL1;
    std::get<1>(motor_controls).stdid = CHASSIS_WHEEL2;
    std::get<2>(motor_controls).stdid = CHASSIS_WHEEL3;
    std::get<3>(motor_controls).stdid = CHASSIS_WHEEL4;

    return true;
}

void OmniDrive::drive_impl(float vx, float vy, float wz) {
    get_motor_feedback();
    calc_motor_outputs(vx, vy, wz);
    send_motor_messages();
}

void OmniDrive::get_motor_feedback() {
    Motor_CAN_ID_t wheel_can_ids[] = {CHASSIS_WHEEL1, CHASSIS_WHEEL2,
                                      CHASSIS_WHEEL3, CHASSIS_WHEEL4};

    mc2::MotorRead motor_read;
    auto message_ts = mc.peek_message(motor_read);
    if (message_ts.has_value()) {
        for (size_t i = 0; i < motor_controls.size(); i++) {
            for (int j = 0; j < MAX_MOTOR_COUNT; j++) {
                if (wheel_can_ids[i] == motor_read.can_ids[j]) {
                    motors.get_raw_feedback(wheel_can_ids[i],
                                            motor_read.feedback[j],
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
    constexpr float inverse_wheel_radius =
        1 / apps_defines::chassis::omni_wheel_radius;
    std::get<0>(motor_angular_vel) =
        (vx + vy +
         wz * (config.chassis_width.get() + config.chassis_length.get()) *
             0.5) *
        inverse_wheel_radius;
    std::get<1>(
        motor_angular_vel) = /* We will put a negative infront of the eq. as motor install is flipped*/
        -((-vx + vy -
           wz * (config.chassis_width.get() + config.chassis_length.get()) *
               0.5) *
          inverse_wheel_radius);
    std::get<2>(
        motor_angular_vel) = /* We will put a negative infront of the eq. as motor install is flipped*/
        -((vx + vy -
           wz * (config.chassis_width.get() + config.chassis_length.get()) *
               0.5) *
          inverse_wheel_radius);
    std::get<3>(motor_angular_vel) =
        (-vx + vy +
         wz * (config.chassis_width.get() + config.chassis_length.get()) *
             0.5) *
        inverse_wheel_radius;
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
        wheel_power_consumption.at(i) =
            torque * angular_speed / 9.55 +
            config.k1 * angular_speed * angular_speed +
            config.k2 * torque * torque + config.a;
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
            allocated_motor_power = state.power_limit.get() / 4;
        } else {
            allocated_motor_power = fabs(motor_angular_vel.at(i)) / vel_sum *
                                    state.power_limit.get() / 2;
        }
        float angular_speed = motor_controls.at(i).feedback.rx_rpm == 0
                                  ? 0.01
                                  : motor_controls.at(i).feedback.rx_rpm;
        float torque = allocated_motor_power * 9.55 / angular_speed;
        float torque_current = torque / OUTPUT_TORQUE_CONSTANT;
        float output_limit =
            fabs(value_limit(torque_current / CURRENT_RESOLUTION,
                             -apps_defines::chassis::motor_max_output,
                             apps_defines::chassis::motor_max_output));
        pid2_set_limits(motor_controls.at(i).f_pid, -output_limit,
                        output_limit);
    }
}

void OmniDrive::calc_motor_volts() {
    calc_power_limits();

    constexpr float RADS_TO_RPM = 60 / (2 * pi);
    for (size_t i = 0; i < motor_controls.size(); i++) {
        int16_t motor_target = value_limit(
            motor_angular_vel.at(i), -apps_defines::chassis::motor_max_output,
            apps_defines::chassis::motor_max_output);
        ramp_set_target(motor_controls.at(i).sp_ramp,
                        motor_controls.at(i).feedback.rx_rpm / RADS_TO_RPM /
                            apps_defines::chassis::wheel_motor_reduction_ratio,
                        motor_target);

        ramp_calc_output(motor_controls.at(i).sp_ramp,
                         config.chassis_app_dt.get());

        pid2_single_loop_control(
            motor_controls.at(i).f_pid,
            motor_controls.at(i).sp_ramp.output * RADS_TO_RPM *
                apps_defines::chassis::wheel_motor_reduction_ratio,
            static_cast<float>(motor_controls.at(i).feedback.rx_rpm),
            ChassisApp<OmniDrive>::get_loop_period());
    }
}

void OmniDrive::calc_motor_outputs(float vx, float vy, float wz) {
    calc_target_motor_speeds(vx, vy, wz);
    calc_motor_volts();
}

void OmniDrive::send_motor_messages() {
    mc2::MotorSet motor_set {};
    for (int i = 0; i < 4; i++) {
        motor_set.motor_can_volts[i] = static_cast<int32_t>(
            std::roundf(motor_controls.at(i).f_pid.total_out));
        motor_set.can_ids[i] =
            static_cast<Motor_CAN_ID_t>(motor_controls.at(i).stdid);
    }

    mc.pub_message(motor_set);
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
    state.power_limit = uarm::strong_types::Watt(new_max_power);
}