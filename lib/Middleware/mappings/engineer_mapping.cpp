#include "mapping_api.hpp"
#include "uarm_lib.hpp"

#ifdef MW_ENABLE_CAN
#include "can.h"
#endif
#ifdef MW_ENABLE_UART
#include "usart.h"
#endif
#ifdef MW_ENABLE_I2C
#include "i2c.h"
#endif
#ifdef MW_ENABLE_SPI
#include "spi.h"
#endif
#ifdef MW_ENABLE_TIM
#include "tim.h"
#endif
#ifdef MW_ENABLE_GPIO
#include "gpio.h"
#endif

namespace platform {
#ifdef MW_ENABLE_UART
    UART_HandleTypeDef* get_hal_uart(MW_UART::Peripheral peripheral) {
        switch (peripheral) {
            case MW_UART::Peripheral::UART_8:
                return &huart8;
            default:
                ASSERT(false, "Unsupported UART peripheral");
                return nullptr;
        }
    }

    MW_UART::Peripheral get_uart_from_hal(UART_HandleTypeDef* huart) {
        if (huart == &huart8) {
            return MW_UART::Peripheral::UART_8;
        }
        ASSERT(false, "Unsupported UART handle");
        return MW_UART::Peripheral::Unknown;
    }
#endif

#ifdef MW_ENABLE_I2C
    I2C_HandleTypeDef* get_hal_i2c(MW_I2C::Periperhal peripheral) {
        (void) peripheral;
        ASSERT(false, "Unsupported I2C peripheral");
        return nullptr;
    }

    MW_I2C::Periperhal get_i2c_from_hal(I2C_HandleTypeDef* hi2c) {
        (void) hi2c;
        ASSERT(false, "Unsupported I2C handle");
        return MW_I2C::Periperhal::Unknown;
    }
#endif

#ifdef MW_ENABLE_CAN
    CAN_HandleTypeDef* get_hal_can(MW_CAN::BUS bus) {
        switch (bus) {
            case MW_CAN::BUS::CAN_1:
            case MW_CAN::BUS::CAN_1B:
                return &hcan1;
            case MW_CAN::BUS::CAN_2:
            case MW_CAN::BUS::CAN_2B:
                return &hcan2;
            default:
                ASSERT(false, "Unsupported CAN bus");
                return nullptr;
        }
    }

    MW_CAN::BUS get_can_from_hal(CAN_HandleTypeDef* hcan) {
        if (hcan == &hcan1) {
            return MW_CAN::BUS::CAN_1;
        } else if (hcan == &hcan2) {
            return MW_CAN::BUS::CAN_2;
        }
        ASSERT(false, "Unsupported CAN handle");
        return MW_CAN::BUS::Unknown;
    }
#endif

#ifdef MW_ENABLE_SPI
    SPI_HandleTypeDef* get_hal_spi(MW_SPI::Peripheral peripheral) {
        (void) peripheral;
        ASSERT(false, "Unsupported SPI peripheral");
        return nullptr;
    }

    MW_SPI::Peripheral get_spi_from_hal(SPI_HandleTypeDef* hspi) {
        (void) hspi;
        ASSERT(false, "Unsupported SPI handle");
        return MW_SPI::Peripheral::Unknown;
    }
#endif

#ifdef MW_ENABLE_TIM
    TIM_HandleTypeDef* get_hal_tim(MW_TIM::Timer timer) {
        (void) timer;
        ASSERT(false, "Unsupported Timer");
        return nullptr;
    }

    uint32_t get_hal_tim_channel(MW_TIM::Channel channel) {
        (void) channel;
        ASSERT(false, "Unsupported TIM channel");
        return 0;
    }
#endif

#ifdef MW_ENABLE_GPIO
    GPIO_TypeDef* get_hal_gpio_port(MW_GPIO::Port port) {
        switch (port) {
            case MW_GPIO::Port::PORT_A:
                return GPIOA;
            case MW_GPIO::Port::PORT_B:
                return GPIOB;
            case MW_GPIO::Port::PORT_C:
                return GPIOC;
            case MW_GPIO::Port::PORT_D:
                return GPIOD;
            case MW_GPIO::Port::PORT_E:
                return GPIOE;
            case MW_GPIO::Port::PORT_F:
                return GPIOF;
            case MW_GPIO::Port::PORT_G:
                return GPIOG;
            case MW_GPIO::Port::PORT_H:
                return GPIOH;
            case MW_GPIO::Port::PORT_I:
                return GPIOI;
            default:
                ASSERT(false, "Invalid GPIO port");
        }
        return nullptr;
    }

    uint16_t get_hal_gpio_pin(MW_GPIO::Pin pin) {
        switch (pin) {
            case MW_GPIO::Pin::PIN_0:
                return GPIO_PIN_0;
            case MW_GPIO::Pin::PIN_1:
                return GPIO_PIN_1;
            case MW_GPIO::Pin::PIN_2:
                return GPIO_PIN_2;
            case MW_GPIO::Pin::PIN_3:
                return GPIO_PIN_3;
            case MW_GPIO::Pin::PIN_4:
                return GPIO_PIN_4;
            case MW_GPIO::Pin::PIN_5:
                return GPIO_PIN_5;
            case MW_GPIO::Pin::PIN_6:
                return GPIO_PIN_6;
            case MW_GPIO::Pin::PIN_7:
                return GPIO_PIN_7;
            case MW_GPIO::Pin::PIN_8:
                return GPIO_PIN_8;
            case MW_GPIO::Pin::PIN_9:
                return GPIO_PIN_9;
            case MW_GPIO::Pin::PIN_10:
                return GPIO_PIN_10;
            case MW_GPIO::Pin::PIN_11:
                return GPIO_PIN_11;
            case MW_GPIO::Pin::PIN_12:
                return GPIO_PIN_12;
            case MW_GPIO::Pin::PIN_13:
                return GPIO_PIN_13;
            case MW_GPIO::Pin::PIN_14:
                return GPIO_PIN_14;
            case MW_GPIO::Pin::PIN_15:
                return GPIO_PIN_15;
            default:
                ASSERT(false, "Invalid GPIO pin");
        }
    }
#endif
}  // namespace platform