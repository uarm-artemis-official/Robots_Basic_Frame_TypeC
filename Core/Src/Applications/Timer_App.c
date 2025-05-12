/*******************************************************************************
* @file           : Timer_App.c
* @brief          : A software timer task to register different periodical task.
* @created time	  : Dec, 2020
* @creator        : AzureRin
*
* @restructed     : Jul, 2023
* @maintainer     : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#include "Timer_App.h"

#include "apps_types.h"
#include "uarm_lib.h"
#include "uarm_os.h"
#include "public_defines.h"
#include "message_center.h"
#include "motors.h"
#include "debug.h"

/**
* @brief  Timer app used to update the CAN data
* 		  Also reserve for other real time tasks
* @param  Not used
* @retval None
*/

static System_Motors_t robot_motors;
static MotorSetMessage_t motor_tx_message;

void Timer_Task_Func(void const * argument){
	/* set task exec period */
	const TickType_t xFrequency = pdMS_TO_TICKS(TIMER_TASK_EXEC_TIME);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	BoardStatus_t status = get_board_status();
	Motor_Config_t config;
	switch (status) {
		case CHASSIS_BOARD: {
			#ifdef SWERVE_CHASSIS
				config = SWERVE;
			#else
				config = DJI_CHASSIS;
			#endif
		}
		case GIMBAL_BOARD: {
			config = DJI_GIMBAL;
		}
		default:
			ASSERT(0, "Unsupported board status in Timer.");
	}

	motors_subsystem_init(&robot_motors, config);
	for (;;){
		uint8_t received_new_message = get_message(MOTOR_SET, &motor_tx_message, 0);
		if (received_new_message == 0) {
			for (int i = 0; i < MAX_MOTOR_COUNT; i++) {
				if (motor_tx_message.can_ids[i] == 0) break;
				if (-30000 <= motor_tx_message.motor_can_volts[i] && motor_tx_message.motor_can_volts[i] <= 30000) {
					set_motor_voltage(&robot_motors, motor_tx_message.can_ids[i], motor_tx_message.motor_can_volts[i]);
				}
			}
		}
		/* CAN data  */
		send_motor_voltage(&robot_motors);

		/* delay until wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
