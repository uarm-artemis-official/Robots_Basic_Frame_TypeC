#include "timers_handler.h"

static TIM_HandleTypeDef* get_timer(Timer_t timer) {
    switch (timer) {
        case AMMO_LID:
            return &htim1;
        default:
            return NULL;
    }
}

static uint32_t get_channel(Timer_Channel_t channel) {
    switch (channel) {
        case CHANNEL_1:
            return TIM_CHANNEL_1;
        case CHANNEL_2:
            return TIM_CHANNEL_2;
        case CHANNEL_3:
            return TIM_CHANNEL_3;
        case CHANNEL_4:
            return TIM_CHANNEL_4;
        case ALL_CHANNELS:
            return TIM_CHANNEL_ALL;
        default:
            // TODO: Error this case.
            return 0;
    }
}

void start_pwm(Timer_t timer, Timer_Channel_t channel) {
    TIM_HandleTypeDef* tim = get_timer(timer);
    uint32_t timer_channel = get_channel(channel);
    HAL_TIM_PWM_Start(tim, timer_channel);
}

void set_pwm_compare_value(Timer_t timer, Timer_Channel_t channel,
                           uint16_t compare_value) {
    TIM_HandleTypeDef* tim = get_timer(timer);
    uint32_t timer_channel = get_channel(channel);
    if (tim) {
        __HAL_TIM_SET_COMPARE(tim, timer_channel, compare_value);
    }
}