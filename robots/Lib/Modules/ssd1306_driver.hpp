#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include "middleware_interfaces.hpp"

namespace ssd1306_driver {

    constexpr uint8_t OLED_ADDRESS = 0x78;
    constexpr uint8_t OLED_WIDTH = 128;
    constexpr uint8_t OLED_HEIGHT = 64;
    constexpr uint8_t CHAR_WIDTH = 6;
    constexpr uint8_t CHAR_HEIGHT = 8;
    constexpr uint8_t MAX_ROWS = OLED_HEIGHT / CHAR_HEIGHT;
    constexpr uint8_t MAX_COLS = OLED_WIDTH / CHAR_WIDTH;

    struct Display_t {
        uint8_t buffer[MAX_ROWS][OLED_WIDTH];
    };

    struct Message_t {
        const char* title;
        const char* body;
    };

    class SSD1306Driver {
       public:
        SSD1306Driver(MW_I2C::II2C& _i2c) : i2c(_i2c) {}

        void init();
        void clear();
        void write_char_to_buffer(Display_t* display, char c,
                                  uint8_t cursor_row, uint8_t cursor_col);
        void write_title(Display_t* display, const Message_t& message,
                         uint8_t offset);
        void write_body(Display_t* display, const Message_t& message,
                        uint8_t line_start);
        void write_buffer_to_gddram(Display_t* display);

        static int get_line_start_index(const char* msg, uint8_t line);
        static uint8_t get_line_count(const char* msg);

       private:
        void send_command(uint8_t command);
        void send_data(uint8_t* data, const size_t size);

        MW_I2C::II2C& i2c;
    };

}  // namespace ssd1306_driver