#ifndef __BUZZER_HPP
#define __BUZZER_HPP

#include <cstdint>
#include "middleware_interfaces.hpp"

/**
 * @brief Buzzer class for playing tones on onboard buzzer using PWM control.
 */
class Buzzer {
   public:
    /**
     * @brief Enum class for musical notes, mapped to their timer autoreload values.
     */
    enum class Note : int {
        G0 = 9523,
        C1 = 7135,
        CS1 = 6733,
        D1 = 6355,
        DS1 = 5999,
        E1 = 5662,
        F1 = 5345,
        FS1 = 5044,
        G1 = 4761,
        GS1 = 4494,
        A1 = 4241,
        AS1 = 4003,
        B1 = 3778,
        C2 = 3566,
        CS2 = 3366,
        D2 = 3177,
        DS2 = 2999,
        E2 = 2830,
        G2 = 2380,
        GS2 = 2246,
        AS2 = 2001
    };

    /**
     * @brief Construct a new Buzzer object.
     * @param pwm Pointer to the middleware PWM interface.
     * @param timer The timer to use for PWM.
     * @param channel The timer channel to use for PWM.
     */
    Buzzer(MW_RTOS::IRTOS& _rtos, MW_TIM::IPWM& _pwm, MW_TIM::Timer _timer,
           MW_TIM::Channel _channel);

    /**
     * @brief Initialize the buzzer (starts PWM).
     */
    void init();

    /**
     * @brief Play a single note for a given duration.
     * @param note Note to play (from Note enum).
     * @param duration_ms Duration in milliseconds.
     * @param compare Duty cycle (default 100).
     */
    void play_note(Note note, uint32_t duration_ms, uint32_t compare = 100);

    /**
     * @brief Play an alarm a given number of times.
     * @param times Number of times to buzz.
     * @param duration_ms Duration of each buzz in milliseconds.
     */
    void alarm_times(uint8_t times, uint16_t duration_ms);

    /**
     * @brief Play the Mario melody.
     * @param bpm Beats per minute for the melody.
     */
    void play_mario(int32_t bpm);

    /**
     * @brief Play the Happy Birthday melody.
     * @param bpm Beats per minute for the melody.
     */
    void play_happy_birthday(int32_t bpm);

    /**
     * @brief Stop the buzzer (set duty cycle to zero).
     */
    void stop();

   private:
    MW_RTOS::IRTOS& rtos;
    MW_TIM::IPWM& pwm;
    MW_TIM::Timer timer;
    MW_TIM::Channel channel;
};

#endif