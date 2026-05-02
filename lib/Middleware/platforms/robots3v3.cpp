#include "mapper.hpp"
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
        (void)peripheral;
        ASSERT(false, "Unimplemented mapper method: get_hal_uart");
        return nullptr;
    }

    MW_UART::Peripheral get_uart_from_hal(UART_HandleTypeDef* huart) {
        (void)huart;
        ASSERT(false, "Unimplemented mapper method: get_uart_from_hal");
        return MW_UART::Peripheral::None;
    }
#endif

#ifdef MW_ENABLE_I2C
    I2C_HandleTypeDef* get_hal_i2c(MW_I2C::Periperhal peripheral) {
        (void)peripheral;
        ASSERT(false, "Unimplemented mapper method: get_hal_i2c");
        return nullptr;
    }

    MW_I2C::Periperhal get_i2c_from_hal(I2C_HandleTypeDef* hi2c) {
        (void)hi2c;
        ASSERT(false, "Unimplemented mapper method: get_i2c_from_hal");
        return MW_I2C::Periperhal::I2C_2;
    }
#endif

#ifdef MW_ENABLE_CAN
    CAN_HandleTypeDef* get_hal_can(MW_CAN::BUS bus) {
        (void)bus;
        ASSERT(false, "Unimplemented mapper method: get_hal_can");
        return nullptr;
    }

    MW_CAN::BUS get_can_from_hal(CAN_HandleTypeDef* hcan) {
        (void)hcan;
        ASSERT(false, "Unimplemented mapper method: get_can_from_hal");
        return MW_CAN::BUS::CAN_1;
    }
#endif

#ifdef MW_ENABLE_SPI
    SPI_HandleTypeDef* get_hal_spi(MW_SPI::Peripheral peripheral) {
        (void)peripheral;
        ASSERT(false, "Unimplemented mapper method: get_hal_spi");
        return nullptr;
    }

    MW_SPI::Peripheral get_spi_from_hal(SPI_HandleTypeDef* hspi) {
        (void)hspi;
        ASSERT(false, "Unimplemented mapper method: get_spi_from_hal");
        return MW_SPI::Peripheral::SPI_1;
    }
#endif

#ifdef MW_ENABLE_TIM
    TIM_HandleTypeDef* get_hal_tim(MW_TIM::Timer timer) {
        switch (timer) {
            case Timer::TIM_1:
                return &htim1;
            case Timer::TIM_4:
                return &htim4;
            case Timer::TIM_5:
                return &htim5;
            case Timer::TIM_8:
                return &htim5;
            case Timer::TIM_10:
                return &htim5;
            case Timer::TIM_13:
                return &htim5;
            default:
                ASSERT(false, "Trying to get unsupported Timer.");
        }
        return nullptr;
    }

    uint32_t get_hal_tim_channel(MW_TIM::Channel channel) {
        (void)channel;
        ASSERT(false, "Unimplemented mapper method: get_hal_tim_channel");
        return 0;
    }
#endif

#ifdef MW_ENABLE_GPIO
    GPIO_TypeDef* get_hal_gpio_port(MW_GPIO::Port port) {
        switch (port) {
            case Port::PORT_A:
                return GPIOA;
            case Port::PORT_B:
                return GPIOB;
            case Port::PORT_C:
                return GPIOC;
            case Port::PORT_D:
                return GPIOD;
            case Port::PORT_E:
                return GPIOE;
            case Port::PORT_F:
                return GPIOF;
            case Port::PORT_G:
                return GPIOG;
            case Port::PORT_H:
                return GPIOH;
            case Port::PORT_I:
                return GPIOI;
            default:
                ASSERT(false, "Invalid GPIO port");
        }
        return nullptr;
    }

    uint16_t get_hal_gpio_pin(MW_GPIO::Pin pin) {
        switch (pin) {
            case Pin::PIN_0:
                return GPIO_PIN_0;
            case Pin::PIN_1:
                return GPIO_PIN_1;
            case Pin::PIN_2:
                return GPIO_PIN_2;
            case Pin::PIN_3:
                return GPIO_PIN_3;
            case Pin::PIN_4:
                return GPIO_PIN_4;
            case Pin::PIN_5:
                return GPIO_PIN_5;
            case Pin::PIN_6:
                return GPIO_PIN_6;
            case Pin::PIN_7:
                return GPIO_PIN_7;
            case Pin::PIN_8:
                return GPIO_PIN_8;
            case Pin::PIN_9:
                return GPIO_PIN_9;
            case Pin::PIN_10:
                return GPIO_PIN_10;
            case Pin::PIN_11:
                return GPIO_PIN_11;
            case Pin::PIN_12:
                return GPIO_PIN_12;
            case Pin::PIN_13:
                return GPIO_PIN_13;
            case Pin::PIN_14:
                return GPIO_PIN_14;
            case Pin::PIN_15:
                return GPIO_PIN_15;
            default:
                ASSERT(false, "Invalid GPIO pin");
        }
    }
#endif
}