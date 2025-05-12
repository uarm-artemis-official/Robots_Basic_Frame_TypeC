#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "subsystems_types.h"


BoardStatus_t get_board_status(void);
void set_led_state(Board_LED_t led, Board_LED_State_t state);

#ifdef __cplusplus
}
#endif

#endif