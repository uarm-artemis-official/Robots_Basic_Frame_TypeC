/*
 * ssd1306_driver.h
 *
 *  Created on: Apr 19, 2025
 *      Author: johnm
 *
 *  This and related files contains driver code displaying simple scrollable text
 * 	messages onto 128x64 OLED screens controlled by SSD1306 drivers (e.g. DFR0650).
 */

#ifndef SRC_DEVICE_OLED_SSD1306_DRIVER_H_
#define SRC_DEVICE_OLED_SSD1306_DRIVER_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "i2c.h"


#define OLED_ADDRESS 0x78
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define CHAR_WIDTH 6  // 5x7 + 1 spacing
#define CHAR_HEIGHT 8 // 8 pixels per page
#define MAX_ROWS (OLED_HEIGHT / CHAR_HEIGHT)
#define MAX_COLS (OLED_WIDTH / CHAR_WIDTH)

extern I2C_HandleTypeDef hi2c2;

#define I2C_DEVICE hi2c2


typedef struct {
	uint8_t buffer[MAX_ROWS][OLED_WIDTH];
} Display_t;


// Message_t data type for outputting messages onto the OLED display.
// newline characters in title field will be replaced with spaces.
// title and body fields will "scroll" horizontally and vertically respectively
// if they are too long for their display areas.
typedef struct {
	char* title;
	char* body;
} Message_t;

int get_line_start_index(char* msg, uint8_t line);
uint8_t get_line_count(char *msg);

void ssd1306_send_command(uint8_t command);
void ssd1306_send_data(uint8_t* data, size_t size);

void ssd1306_init();
void ssd1306_clear(void);
void ssd1306_write_char_to_buffer(Display_t* display, char c, uint8_t cursor_row, uint8_t cursor_col);
void ssd1306_write_title(Display_t* display, Message_t message, uint8_t offset);
void ssd1306_write_body(Display_t* display, Message_t message, uint8_t line_start);
void ssd1306_write_buffer_to_gddram(Display_t* display);

#endif /* SRC_DEVICE_OLED_SSD1306_DRIVER_H_ */
