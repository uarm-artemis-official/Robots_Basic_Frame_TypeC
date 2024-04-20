/*******************************************************************************
* @file           : self_check.c
* @brief          : self check system for initialization
* @created time	  : Aug, 2023
* @author         : Haoran
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __SELF_CHECK_C__
#define __SELF_CHECK_C__

#include "self_check.h"
#include "public_defines.h"
#include "motor.h"
#include "main.h"

/* define internal vars */

/* define internal functions */

/* extern globe vars */
extern Motor motor_data[MOTOR_COUNT];

#define ONE_SECOND_CNT 1000
#define TWO_SECOND_CNT 2000

// TODO: Register system for self-check
/**
  * @brief    self-check system mian func
  * @param[in] motor id to be checked
  * @retval    SelfCheckStatus_t
  */
SelfCheckStatus_t self_check_system(){
	// Checks motors in motor_ids are giving feedback.
	uint8_t num_of_motors;
	uint8_t motor_ids[8];

	// Set motor_ids of motors controlled by each board.
	if(board_status == CHASSIS_BOARD){
		num_of_motors = 4;
		motor_ids[0] = wheel_id1;
		motor_ids[1] = wheel_id2;
		motor_ids[2] = wheel_id3;
		motor_ids[3] = wheel_id4;
	}
	else if(board_status == GIMBAL_BOARD){
		num_of_motors = 3;
		motor_ids[0] = yaw_id;
		motor_ids[1] = pitch_id;
		motor_ids[2] = mag_2006_id;
	}

	// Check motors in motor_ids.
	for (int i = 0; i < num_of_motors; i++) {
		if (self_check_motors(motor_ids[i]) == CHECK_FAIL) {
#ifndef SILENT_SELF_CHECK
			buzzer_alarm_times(i + 1, TWO_SECOND_CNT, &buzzer);
#endif
			return CHECK_FAIL;
		}
	}
	return CHECK_OK;
}


/**
  * @brief     self check DJI CAN motor status based on feedback
  * @param[in] motor id to be checked
  * @retval    SelfCheckStatus_t
  */
SelfCheckStatus_t self_check_motors(uint8_t motor_id){
	/* check motor online status */
	if(motor_data[motor_id].motor_feedback.rx_angle > 0){
		/* this may have issue when the fb angle exactly equal to 0 (very rare)*/
		return 	CHECK_OK;
	}
	return CHECK_FAIL;
}

#endif /* __SELF_CHECK_C__ */
