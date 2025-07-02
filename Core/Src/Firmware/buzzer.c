/*******************************************************************************
* @file           : buzzer.c
* @brief          : Use spreadsheet to add notes to the autoreload values, they correspond to PWM frequency
*      			    Set_compare values correspond to duty cycle, aka volume
* @created time	  : Dec 5, 2020
* @author         : Adan Wang
*
* @re-struct on   : Aug, 2023
* co-author		  : Haoran Qi
******************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#include "buzzer.h"



static Buzzer_t buzzer;
/*
 * brief play different times for checking purposes
 * param[in] times want to be buzzed
 * param[in] delay duration
 * retval None
 * author Haoran
 */
void buzzer_init(Buzzer_t *buzz){
	buzz->buzz_times = 0;
	buzz->buzzer_tick = 0;
	buzz->times_tick  =0;
	HAL_TIM_PWM_Start(&BUZZ_HTIM, BUZZ_PWM_CH);
}

void buzzer_set_tune(uint16_t tune, uint16_t ctrl){
    /* set Auto-reload value for the timer */
    __HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM, tune);
    /* set compare value to control duty cycle */
    __HAL_TIM_SET_COMPARE(&BUZZ_HTIM, BUZZ_PWM_CH, ctrl);
}

/*
 * brief play different times for checking purposes
 * param[in] times want to be buzzed
 * param[in] delay duration
 * retval None
 * author Haoran
 */
void buzzer_alarm_times(uint8_t times, uint16_t duration, Buzzer_t *buzz){
	/* check if a period of alarm has been called */
	uint32_t cur_ticks = HAL_GetTick();
    if(cur_ticks - buzz->buzzer_tick > duration){
        buzz->buzzer_tick = cur_ticks;
        buzz->times_tick = cur_ticks;
        buzz->buzz_times = times; // Set the number of times buzzer should play
    }
    /* start buzzering */
    else if(buzz->buzz_times != 0){
    	/* phase 1: 100ms beep once */
        if(cur_ticks - buzz->times_tick < 200){			     //use Hal_GetTick to use for both
            buzzer_set_tune(7135, 100); // play the tone c1  //in freertos and normal program
        }
        /* phase 2: then stop beeping for 200ms(total 300ms) */
        else if(cur_ticks - buzz->times_tick < 400){
            buzzer_set_tune(7135, 0);  // silence the buzzer
        }
        /* phase 3: buzz time minus 1 */
        else{
            buzz->buzz_times--;
            buzz->times_tick = cur_ticks;
        }
    }
}


/* below are the buzzer tune play functions, used for imu task and for fun! */
void buzzer_play_g0(int32_t duration){
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 35);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,9523);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_c1(int32_t duration){
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 35);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,7135);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}


void buzzer_play_cs1(int32_t duration){
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 35);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,6733);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_d1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 28);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,6355);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_ds1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 28);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,5999);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_e1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 25);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,5662);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_f1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 24);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,5345);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_fs1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 24);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,5044);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_g1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 21);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,4761);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}


void buzzer_play_gs1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 21);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,4494);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_a1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 19);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,4241);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_as1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 19);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,4003);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_b1(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 17);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,3778);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_c2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,3566);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_cs2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,3366);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_d2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,3177);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_ds2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,2999);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_e2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,2830);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_as2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,2001);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_g2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,2380);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_play_gs2(int32_t duration){
	//__HAL_TIM_PRESCALER(&BUZZ_HTIM, 16);
	__HAL_TIM_SET_COUNTER(&BUZZ_HTIM,0);
	__HAL_TIM_SET_AUTORELOAD(&BUZZ_HTIM,2246);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,100);
	osDelay(duration);
	__HAL_TIM_SET_COMPARE(&BUZZ_HTIM,BUZZ_PWM_CH,0);
}

void buzzer_rest(int32_t duration){
	osDelay(duration);
}

void buzzer_stop(void) {
    __HAL_TIM_SET_COMPARE(&BUZZ_HTIM, BUZZ_PWM_CH, 0);
}

void buzzer_play_scale(int32_t duration){
	buzzer_play_c1(duration);
	buzzer_play_d1(duration);
	buzzer_play_e1(duration);
	buzzer_play_f1(duration);
	buzzer_play_g1(duration);
	buzzer_play_a1(duration);
	buzzer_play_b1(duration);
	buzzer_play_c2(duration);
}

void buzzer_play_chromatic(int32_t duration){
	buzzer_play_c1(duration);
	buzzer_play_cs1(duration);
	buzzer_play_d1(duration);
	buzzer_play_ds1(duration);
	buzzer_play_e1(duration);
	buzzer_play_f1(duration);
	buzzer_play_fs1(duration);
	buzzer_play_g1(duration);
	buzzer_play_gs1(duration);
	buzzer_play_a1(duration);
	buzzer_play_as1(duration);
	buzzer_play_b1(duration);
	buzzer_play_c2(duration);
}


void buzzer_play_mario(int32_t bpm){
	int32_t quarter=(double)60/bpm*1000;
	int32_t eighth=(double)60/bpm*1000*0.5;

	buzzer_play_e1(eighth);
	buzzer_play_e1(eighth);
	buzzer_rest(eighth);
	buzzer_play_e1(eighth);
	buzzer_rest(eighth);
	buzzer_play_c1(eighth);
	buzzer_play_e1(quarter);
	buzzer_play_g1(quarter);
	buzzer_rest(quarter);
	buzzer_play_g0(quarter);
}

void buzzer_play_ynk(int32_t bpm){
	int32_t quarter=(double)60/bpm*1000;
	int32_t eighth=(double)60/bpm*1000*0.5;
	int32_t eighth_dot=(double)60/bpm*1000*0.5*1.5;
	int32_t sixteenth=(double)60/bpm*0.25*1000;

	/* end of m41 */
		buzzer_play_ds2(eighth);
		buzzer_play_d2(eighth);
		buzzer_play_as1(eighth);

		/*m42*/
		buzzer_play_as1(eighth_dot);
		buzzer_play_c2(eighth_dot);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_c2(eighth);

		/*m43*/
		buzzer_play_as1(eighth);
		buzzer_play_f1(sixteenth);
		buzzer_play_g1(eighth);
		buzzer_play_ds1(sixteenth);
		buzzer_play_f1(sixteenth);
		buzzer_play_ds1(quarter);
		buzzer_play_ds1(eighth);
		buzzer_play_as1(eighth);

		/*m44*/
		buzzer_play_gs1(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_d1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_gs1(eighth);

		/*m45*/
		buzzer_play_g1(eighth);
		buzzer_play_f1(sixteenth);
		buzzer_play_g1(eighth_dot);
		buzzer_play_c2(eighth);
		buzzer_play_as1(quarter);
		buzzer_play_g1(eighth);
		buzzer_play_as1(eighth);

		//m46
		buzzer_play_c2(eighth_dot);
		buzzer_play_gs1(eighth_dot);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_c2(eighth);

		//m47
		buzzer_play_as1(eighth);
		buzzer_play_c2(sixteenth);
		buzzer_play_g1(eighth_dot);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_rest(eighth);
		buzzer_play_c1(eighth);
		buzzer_play_d1(eighth);

		//m48
		buzzer_play_ds1(eighth);
		buzzer_play_c1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_c1(eighth);

		//m49
		buzzer_play_b1(eighth);
		buzzer_rest(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_ds2(eighth);
		buzzer_play_d2(eighth);
		buzzer_play_as1(eighth);

		//m50
		buzzer_play_as1(eighth_dot);
		buzzer_play_c2(eighth_dot);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_c2(eighth);

		//m51
		buzzer_play_as1(eighth);
		buzzer_play_c2(sixteenth);
		buzzer_play_g1(eighth_dot);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_rest(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_as1(eighth);

		//m52
		buzzer_play_gs1(eighth);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_d1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_gs1(eighth);

		//m53
		buzzer_play_g1(eighth);
		buzzer_play_f1(sixteenth);
		buzzer_play_g1(eighth_dot);
		buzzer_play_c2(eighth);
		buzzer_play_as1(quarter);
		buzzer_play_g1(eighth);
		buzzer_play_as1(eighth);

		//m54
		buzzer_play_c2(eighth_dot);
		buzzer_play_gs1(eighth_dot);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_d2(eighth);
		buzzer_play_c2(eighth);
		buzzer_play_as1(eighth);

		//m55
		buzzer_play_as1(eighth);
		buzzer_play_c2(eighth);
		buzzer_play_d2(eighth);
		buzzer_play_ds2(quarter);
		buzzer_play_g1(eighth);
		buzzer_play_f1(eighth);
		buzzer_play_ds1(eighth);

		//m56
		buzzer_rest(eighth);
		buzzer_play_c1(eighth);
		buzzer_play_ds1(eighth);
		buzzer_play_gs1(sixteenth);
		buzzer_play_g1(eighth);
		buzzer_rest(sixteenth);
		buzzer_play_ds1(eighth);
}

void play_happy_birthday(int32_t bpm){
	int32_t quarter = (double)60 / bpm * 1000;
	int32_t eighth = (double)60 / bpm * 1000 * 0.5;
	int32_t half = (double)60 / bpm * 2 * 1000;

    // Happy Birthday song structure
	buzzer_play_c1(quarter);
	buzzer_play_c1(eighth);
	buzzer_play_d1(eighth);
	buzzer_play_c1(quarter);
	buzzer_play_f1(quarter);
	buzzer_play_e1(half);
	buzzer_rest(eighth);

	buzzer_play_c1(quarter);
	buzzer_play_c1(eighth);
	buzzer_play_d1(eighth);
	buzzer_play_c1(quarter);
	buzzer_play_g1(quarter);
	buzzer_play_f1(half);
	buzzer_rest(eighth);

	buzzer_play_c1(quarter);
	buzzer_play_c1(quarter);
	buzzer_play_c2(quarter);
	buzzer_play_a1(quarter);
	buzzer_play_f1(quarter);
	buzzer_play_e1(quarter);
	buzzer_play_d1(quarter + eighth); // Extended a bit to match the rhythm
	buzzer_rest(eighth);

	buzzer_play_as1(quarter);
	buzzer_play_as1(quarter);
	buzzer_play_a1(quarter);
	buzzer_play_f1(quarter);
	buzzer_play_g1(quarter);
	buzzer_play_f1(half);
}
