#ifndef __TIMERS_H
#define __TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

typedef enum {
    AMMO_LID,
} Timer_t;

typedef enum {
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    ALL_CHANNELS,
} Timer_Channel_t;

void start_pwm(Timer_t timer, Timer_Channel_t channel);
void set_pwm_compare_value(Timer_t timer, Timer_Channel_t channel,
                           uint16_t compare_value);

#ifdef __cplusplus
}
#endif

#endif