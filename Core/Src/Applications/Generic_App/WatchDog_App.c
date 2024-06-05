/*******************************************************************************
* @file           : WatchDog_App.c
* @brief          : An independent watch dog program monitoring the system status
* 					in real time.
* @created time	  : Aug, 2023
* @author         : Haoran
*********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

/* IWDG is default to reset the system per 2s if not feeding dog */

#ifndef __WATCHDOG_APP_C__
#define __WATCHDOG_APP_C__

#include "WatchDog_App.h"
#include "Referee_App.h"
#include "main.h"
#include "usart.h"

extern uint8_t ref_tx_frame[MAX_REF_TX_DATA_LEN];
extern UART_HandleTypeDef huart1;
extern Referee_t referee;


/* define global watch dog Semaphore here for blocking */
SemaphoreHandle_t wdgSemaphore;

/**
  * @brief     watch dog task main entry function
  * @retval    None
  */
/* Task execution time (per loop): 100 ms */
void WatchDog_Task_Function(void){
	/* define wd global flag */
	static uint8_t wd_daemon_flag = 0;

	/* init the watch dog program */
	wdg_task_init();

	/* set task exec period */
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(150); // task exec period 1ms

	/* init the task ticks */
	xLastWakeTime = xTaskGetTickCount();

	referee.cur_sending_id = UI_INFANTRY_MARK;

	/* main imu task begins */
	for(;;){

		/* self check progress*/
		if(self_check_system() == CHECK_OK)
			wd_daemon_flag = 0;//pass
		else
			wd_daemon_flag = 1;//fail

		/* Put UI transmit here temporarily */
		switch(referee.cur_sending_id){
			case UI_ROBOT_ACT_MODE:{
				referee_set_ui_data(&referee, UI_ROBOT_ACT_MODE);
				referee_send_ui_data(INTERA_UI_ID, (uint8_t *)&referee.ui_custom_data, UI_ACT_MODE_LEN);
				break;
			}
			case UI_HERO_MARK: {
				referee_set_ui_data(&referee, UI_HERO_MARK);
				referee_send_ui_data(INTERA_UI_ID, (uint8_t *)&referee.ui_intrect_data, UI_HERO_MARK_LEN);
				break;
			}
			case UI_INFANTRY_MARK: {
				referee_set_ui_data(&referee, UI_INFANTRY_MARK);
				referee_send_ui_data(INTERA_UI_ID, (uint8_t *)&referee.ui_intrect_data, UI_INFANTRY_MARK_LEN);
				break;
						}
			default:break;
		}

		if(referee.cur_sending_id == UI_INFANTRY_MARK)
			referee.cur_sending_id = UI_ROBOT_ACT_MODE;
		else
			referee.cur_sending_id = UI_INFANTRY_MARK;


#ifdef USE_IWDG
		wdg_daemon_feed_dog();
		if(wd_daemon_flag == 0){
			 xSemaphoreGive(wdgSemaphore); // release the semaphore if self-check is okay
			 wd_daemon_feed_dog();
		}
		else{
			xSemaphoreTake(wdgSemaphore, portMAX_DELAY); // lock the semaphore if self-check fails
		}

		/* or feed dog */
		//FIXME: We should only feed dog when self-check is okay,
		//       but currently just feed dog whatever the check pass or fail
#endif
		/* delay utill wake time */
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

	}
}

void wdg_task_init(void){
	/* set a binary watch dog semaphore */
	wdgSemaphore = xSemaphoreCreateBinary();
}

void wdg_daemon_feed_dog(void){
	/* refresh the watchdog timer */
	HAL_IWDG_Refresh(&hiwdg);
}

void wdg_daemon_reset_system(){
	/* reset system by calling HAL */

}

#endif /* __WATCHDOG_APP_C__ */
