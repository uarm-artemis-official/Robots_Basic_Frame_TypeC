#include "middleware_classes.hpp"
#include "lk_motor_driver.hpp"

static MW_GPIO::GPIO gpio;
static MW_BASE::Base base;
static MW_UART::UART uart;


bool init_firmware() {
    bool gpio_status = gpio.init();
    bool base_status = base.init();
    return gpio_status && base_status;
}

void main_cpp() {
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_1,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_3,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_5,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_7,
                   MW_GPIO::State::HIGH);

    std::array<std::byte, 5> msg;
    std::span<std::byte, 5> msg_span(msg);
    lk_motor::rs485::format_read_motor_state_1(1, msg_span);
    while (true) {
        uart.send_data(MW_UART::Peripheral::UART8, msg_span.data(), 10);
        base.delay_ms(1000);
    }
}