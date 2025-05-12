#include "motors.h"
#include "subsystems_defines.h"
#include "dji_motor.h"
#include "lk_motor.h"
#include "uarm_lib.h"


void motors_subsystem_init(System_Motors_t *system, Motor_Config_t config) {
	memset(system, 0, sizeof(System_Motors_t));

	system->config = config;
	switch (config) {
		case DJI_GIMBAL: {
			system->motors[0] = (Generic_Motor_t) { .feedback_id = SHOOT_LEFT_FRIC, .brand = DJI };
			system->motors[1] = (Generic_Motor_t) { .feedback_id = SHOOT_RIGHT_FRIC, .brand = DJI };
			system->motors[2] = (Generic_Motor_t) { .feedback_id = GIMBAL_YAW, .brand = DJI };
			system->motors[3] = (Generic_Motor_t) { .feedback_id = GIMBAL_PITCH, .brand = DJI };
			system->motors[4] = (Generic_Motor_t) { .feedback_id = SHOOT_LOADER, .brand = DJI };
			break;
		}
		case DJI_CHASSIS: {
			system->motors[0] = (Generic_Motor_t) { .feedback_id = CHASSIS_WHEEL1, .brand = DJI };
			system->motors[1] = (Generic_Motor_t) { .feedback_id = CHASSIS_WHEEL2, .brand = DJI };
			system->motors[2] = (Generic_Motor_t) { .feedback_id = CHASSIS_WHEEL3, .brand = DJI };
			system->motors[3] = (Generic_Motor_t) { .feedback_id = CHASSIS_WHEEL4, .brand = DJI };
			break;
		}
		case SWERVE: {
			// TODO
			break;
		}
		default:
			ASSERT(0, "Attempt to configure subsystems::motors to unknown configuration.");
	}
}


void parse_feedback(uint32_t stdid, uint8_t data[8], Motor_Feedback_t *feedback) {
	if (0x200 < stdid && stdid < 0x212) {
		dji_motor_parse_feedback(data, feedback);
	} else if (0 < stdid && stdid < 0x140) { // TODO: Change to valid limits
		lk_motor_parse_feedback(data, feedback);
	} else {
		ASSERT(0, "subsystems::motors cannot parse feedback for data with unsupported stdid");
	}
}


void set_motor_voltage(System_Motors_t *system, Motor_CAN_ID_t can_id, int32_t output) {	
	for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
		if (system->motors[i].feedback_id == can_id) {
			system->motors[i].tx_data = output;
			break;
		}
	}
}

void send_motor_voltage(System_Motors_t *system) {
	switch (system->config) {
		case DJI_GIMBAL:
			dji_motor_send((int32_t) GM6020, system->motors[2].tx_data, system->motors[3].tx_data, system->motors[4].tx_data, 0);
			break;
		case DJI_CHASSIS:
			dji_motor_send((int32_t) M3508, system->motors[0].tx_data, system->motors[1].tx_data, system->motors[2].tx_data, system->motors[3].tx_data);
			break;
		case SWERVE:
			// TODO
			break;
		default:
			ASSERT(0, "Attempt to send for an unknown motors configuration.");
	}
}