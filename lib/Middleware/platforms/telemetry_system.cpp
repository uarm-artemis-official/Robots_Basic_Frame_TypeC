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
        (void)timer;
        ASSERT(false, "Unimplemented mapper method: get_hal_tim");
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
        (void)port;
        ASSERT(false, "Unimplemented mapper method: get_hal_gpio_port");
        return nullptr;
    }

    uint16_t get_hal_gpio_pin(MW_GPIO::Pin pin) {
        (void)pin;
        ASSERT(false, "Unimplemented mapper method: get_hal_gpio_pin");
        return 0;
    }
#endif
}