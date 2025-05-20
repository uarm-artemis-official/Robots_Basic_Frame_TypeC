#ifndef __DEBUG_H
#define __DEBUG_H

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class Debug : public IDebug {
   public:
    BoardStatus_t get_board_status(void) override;
    void set_led_state(Board_LED_t led, Board_LED_State_t state) override;
};

#endif