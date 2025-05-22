#include "Omni_Drive.h"
#include <cstring>
#include "motors.h"
#include "pid.h"
#include "public_defines.h"

void OmniDrive::init_impl() {
    wheel_speed[4] = {0};

    for (int i = 0; i < CHASSIS_MAX_WHEELS; i++) {
        std::memset(&(motor_controls[i]), 0, sizeof(Chassis_Wheel_Control_t));
        pid_param_init(&(motor_controls[i].f_pid), max_out_wheel,
                       max_I_out_wheel, max_err_wheel, kp_wheel, ki_wheel,
                       kd_wheel);
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
                    Motors::parse_feedback(wheel_can_ids[i],
                                           read_message.feedback[i],
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

    wheel_speed[CHASSIS_WHEEL1_CAN_ID] = static_cast<int16_t>(
        (vx + vy +
         wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) * 0.5) *
        (1 / CHASSIS_OMNI_WHEEL_RADIUS) * CHASSIS_MOTOR_DEC_RATIO);
    wheel_speed
        [CHASSIS_WHEEL2_CAN_ID] = /* We will put a negative infront of the eq. as motor install is flipped*/
        static_cast<int16_t>(
            -((-vx + vy -
               wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) * 0.5) *
              (1 / CHASSIS_OMNI_WHEEL_RADIUS) * CHASSIS_MOTOR_DEC_RATIO));
    wheel_speed
        [CHASSIS_WHEEL3_CAN_ID] = /* We will put a negative infront of the eq. as motor install is flipped*/
        static_cast<int16_t>(
            -((vx + vy -
               wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) * 0.5) *
              (1 / CHASSIS_OMNI_WHEEL_RADIUS) * CHASSIS_MOTOR_DEC_RATIO));
    wheel_speed[CHASSIS_WHEEL4_CAN_ID] = static_cast<int16_t>(
        (-vx + vy +
         wz * (CHASSIS_WHEEL_X_LENGTH + CHASSIS_WHEEL_Y_LENGTH) * 0.5) *
        (1 / CHASSIS_OMNI_WHEEL_RADIUS) * CHASSIS_MOTOR_DEC_RATIO);
}

void OmniDrive::calc_motor_volts() {
    for (int i = 0; i < 4; i++) {
        VAL_LIMIT(wheel_speed[i], -CHASSIS_MAX_SPEED, CHASSIS_MAX_SPEED);
        pid_single_loop_control(
            static_cast<float>(wheel_speed[i]), &(motor_controls[i].f_pid),
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