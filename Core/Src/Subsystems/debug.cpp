#include "debug.h"

#include "uarm_lib.h"

#include "gpio.h"

BoardStatus_t get_board_status(void) {
    if (HAL_GPIO_ReadPin(Board_Status_GPIO_Port, Board_Status_Pin) ==
        GPIO_PIN_RESET) {
        return CHASSIS_BOARD;
    } else {
        return GIMBAL_BOARD;
    }
}

void set_led_state(Board_LED_t led, Board_LED_State_t state) {
    uint16_t port, pin_state;
    switch (led) {
        case BLUE:
            port = GPIO_PIN_10;
            break;
        case RED:
            port = GPIO_PIN_12;
            break;
        case GREEN:
            port = GPIO_PIN_11;
            break;
        default:
            ASSERT(0, "Cannot set unknown board LED.");
    }

    switch (state) {
        case ON:
            pin_state = GPIO_PIN_SET;
            break;
        case OFF:
            pin_state = GPIO_PIN_RESET;
            break;
        default:
            ASSERT(0, "Cannot set board LED to unknown state.");
    }

    HAL_GPIO_WritePin(GPIOH, port, static_cast<GPIO_PinState>(pin_state));
}