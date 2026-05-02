#include "middleware_classes.hpp"

static MW_GPIO::GPIO gpio;
static MW_BASE::Base base;

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

    while (true) {
        base.delay_ms(1);
    }
}