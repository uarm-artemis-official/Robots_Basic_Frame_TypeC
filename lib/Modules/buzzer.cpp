#include "buzzer.hpp"

Buzzer::Buzzer(MW_RTOS::IRTOS& _rtos, MW_TIM::IPWM& _pwm, MW_TIM::Timer _timer,
               MW_TIM::Channel _channel)
    : rtos(_rtos), pwm(_pwm), timer(_timer), channel(_channel) {}

void Buzzer::init() {
    pwm.start(timer, channel);
}

void Buzzer::play_note(Note note, uint32_t duration_ms, uint32_t compare) {
    pwm.set_counter(timer, 0);
    pwm.set_autoreload(timer, static_cast<uint32_t>(note));
    pwm.set_compare(timer, channel, compare);
    rtos.delay_ms(duration_ms);
    pwm.set_compare(timer, channel, 0);
}

void Buzzer::alarm_times(uint8_t times, uint16_t duration_ms) {
    for (uint8_t i = 0; i < times; ++i) {
        // Phase 1: 100ms beep
        play_note(Note::C1, duration_ms, 100);
        // Phase 2: silence for 200ms
        pwm.set_compare(timer, channel, 0);
        rtos.delay_ms(200);
    }
}

void Buzzer::play_mario(int32_t bpm) {
    int32_t quarter = static_cast<int32_t>(60.0 / bpm * 1000);
    int32_t eighth = static_cast<int32_t>(60.0 / bpm * 1000 * 0.5);

    play_note(Note::E1, eighth);
    play_note(Note::E1, eighth);
    rtos.delay_ms(eighth);
    play_note(Note::E1, eighth);
    rtos.delay_ms(eighth);
    play_note(Note::C1, eighth);
    play_note(Note::E1, quarter);
    play_note(Note::G1, quarter);
    rtos.delay_ms(quarter);
    play_note(Note::G0, quarter);
}

void Buzzer::play_happy_birthday(int32_t bpm) {
    int32_t quarter = static_cast<int32_t>(60.0 / bpm * 1000);
    int32_t eighth = static_cast<int32_t>(60.0 / bpm * 1000 * 0.5);
    int32_t half = static_cast<int32_t>(60.0 / bpm * 2 * 1000);

    // Happy Birthday song structure
    play_note(Note::C1, quarter);
    play_note(Note::C1, eighth);
    play_note(Note::D1, eighth);
    play_note(Note::C1, quarter);
    play_note(Note::F1, quarter);
    play_note(Note::E1, half);
    rtos.delay_ms(eighth);

    play_note(Note::C1, quarter);
    play_note(Note::C1, eighth);
    play_note(Note::D1, eighth);
    play_note(Note::C1, quarter);
    play_note(Note::G1, quarter);
    play_note(Note::F1, half);
    rtos.delay_ms(eighth);

    play_note(Note::C1, quarter);
    play_note(Note::C1, quarter);
    play_note(Note::C2, quarter);
    play_note(Note::A1, quarter);
    play_note(Note::F1, quarter);
    play_note(Note::E1, quarter);
    play_note(Note::D1, quarter + eighth);
    rtos.delay_ms(eighth);

    play_note(Note::AS1, quarter);
    play_note(Note::AS1, quarter);
    play_note(Note::A1, quarter);
    play_note(Note::F1, quarter);
    play_note(Note::G1, quarter);
    play_note(Note::F1, half);
}

void Buzzer::stop() {
    pwm.set_compare(timer, channel, 0);
}