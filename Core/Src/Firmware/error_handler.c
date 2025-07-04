/*
 * error_handler.c
 *
 *  Created on: Mar 19, 2025
 *      Author: johnm
 */

#include "error_handler.h"

// Called when ASSERTS or similar checks fail somewhere in the codebase.
// This method is meant to disable all bot functionality and disable a message on a
// connected OLED display.
//
// Future Work: make fail-safe function/add ability to recover from certain error states.
void error_handler(const char* msg) {
    __disable_irq();
    ssd1306_init();

    Display_t display;
    Message_t message = {
        .title = "ASSERT ERROR!",
        .body = msg,
    };

    uint8_t title_offset = 0;
    uint8_t line = 0;
    uint8_t body_line_count = get_line_count(msg);
    int title_length = strlen(message.title);
    while (1) {
        ssd1306_write_title(&display, message, title_offset);
        ssd1306_write_body(&display, message, line);

        title_offset = (title_offset + 1) % title_length;
        line = (line + 1) % body_line_count;
        dwt_sleep(300);
    }
}
