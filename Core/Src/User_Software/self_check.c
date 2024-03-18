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
	/* motor check processes */

	if(board_status == CHASSIS_BOARD){
		/*
		if(self_check_motors(wheel_id1) == CHECK_FAIL){
			buzzer_alarm_times(1, TWO_SECOND_CNT, &buzzer);// 1 buzz per second
			return CHECK_FAIL;
		}
		else if(self_check_motors(wheel_id2) == CHECK_FAIL){
			buzzer_alarm_times(2, TWO_SECOND_CNT, &buzzer);// 2 buzz per second
			return CHECK_FAIL;
		}
		else if(self_check_motors(wheel_id3) == CHECK_FAIL){
			buzzer_alarm_times(3, TWO_SECOND_CNT, &buzzer);// 3 buzz per second
			return CHECK_FAIL;
		}
		else if(self_check_motors(wheel_id4) == CHECK_FAIL){
			buzzer_alarm_times(4, TWO_SECOND_CNT, &buzzer);// 4 buzz per second
			return CHECK_FAIL;
		}
		*/
	}
	else if(board_status == GIMBAL_BOARD){
//		if(self_check_motors(yaw_id) == CHECK_FAIL){
//			buzzer_alarm_times(1, TWO_SECOND_CNT, &buzzer);// 1 buzz per second
//			return CHECK_FAIL;
//		}
//		else if(self_check_motors(pitch_id) == CHECK_FAIL){
//			buzzer_alarm_times(2, TWO_SECOND_CNT, &buzzer);// 2 buzz per second
//			return CHECK_FAIL;
//		}
//		else if(self_check_motors(mag_2006_id) == CHECK_FAIL){
//			buzzer_alarm_times(3, TWO_SECOND_CNT, &buzzer);// 3 buzz per second
//			return CHECK_FAIL;
//		}
	}
	/* RC check processes */
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
