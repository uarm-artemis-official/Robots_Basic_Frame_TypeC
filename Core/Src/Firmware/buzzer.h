/*******************************************************************************
* @file           : buzzer.h
* @brief          : Use spreadsheet to add notes to the autoreload values, they correspond to PWM frequency
*      			    Set_compare values correspond to duty cycle, aka volume
* @created time	  : Dec 5, 2020
* @author         : Adan Wang
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __BUZZER_H__
#define __BUZZER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f407xx.h"
#include "cmsis_os.h"
#include "tim.h"

//extern TIM_HandleTypeDef htim12;

#define BUZZ_HTIM htim4
#define BUZZ_PWM_CH TIM_CHANNEL_3

typedef struct{
	uint32_t buzzer_tick;
	uint32_t times_tick;

	uint8_t buzz_times;
}Buzzer_t;


/* use for alarm */
void buzzer_init(Buzzer_t *buzz);
void buzzer_set_tune(uint16_t tune, uint16_t ctrl);
void buzzer_alarm_times(uint8_t times, uint16_t duration, Buzzer_t *buzz);

void buzzer_stop(void);
void buzzer_play_scale(int32_t duration);
void buzzer_play_chromatic(int32_t duration);
void buzzer_play_mario(int32_t bpm);
void play_happy_birthday(int32_t bpm);
void buzzer_play_c1(int32_t duration);

#ifdef __cplusplus
}
#endif

#endif /* SRC_BUZZER_H_ */
