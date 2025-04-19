/*
 * error_handler.c
 *
 *  Created on: Mar 19, 2025
 *      Author: johnm
 */

#include "error_handler.h"


void error_handler(char *msg) {
	// TODO: Add interface to output msg to OLED display on bot.
	// Error handler should probably have an init function that
	// checks if the OLED display is connected and can be communicated with.
	// Fall back indicators: buzzer or led.

	__disable_irq();
	while (1) {
		asm volatile("nop");
	}
}
