#ifndef __MAPPER_HPP
#define __MAPPER_HPP

#include "middleware_types.hpp"

#if defined(STM32F407XX) || defined(STM32F427XX)
#include "stm32f4xx_hal.h"
#else
#error \
    "Unsupported STM32 series. Please implement the necessary HAL type mappings in mapper.hpp for your specific STM32 series."
#endif

namespace platform {
#ifdef MW_ENABLE_UART
#include "stm32f4xx_hal_uart.h"
    UART_HandleTypeDef* get_hal_uart(MW_UART::Peripheral peripheral);
    MW_UART::Peripheral get_uart_from_hal(UART_HandleTypeDef* huart);
#endif

#ifdef MW_ENABLE_I2C
#include "stm32f4xx_hal_i2c.h"
    I2C_HandleTypeDef* get_hal_i2c(MW_I2C::Periperhal peripheral);
    MW_I2C::Periperhal get_i2c_from_hal(I2C_HandleTypeDef* hi2c);
#endif

#ifdef MW_ENABLE_CAN
#include "stm32f4xx_hal_can.h"
    CAN_HandleTypeDef* get_hal_can(MW_CAN::BUS bus);
    MW_CAN::BUS get_can_from_hal(CAN_HandleTypeDef* hcan);
#endif

#ifdef MW_ENABLE_SPI
#include "stm32f4xx_hal_spi.h"
    SPI_HandleTypeDef* get_hal_spi(MW_SPI::Peripheral peripheral);
    MW_SPI::Peripheral get_spi_from_hal(SPI_HandleTypeDef* hspi);
#endif

#ifdef MW_ENABLE_TIM
#include "stm32f4xx_hal_tim.h"
    TIM_HandleTypeDef* get_hal_tim(MW_TIM::Timer timer);
    uint32_t get_hal_tim_channel(MW_TIM::Channel channel);
#endif

#ifdef MW_ENABLE_GPIO
#include "stm32f4xx_hal_gpio.h"
    GPIO_TypeDef* get_hal_gpio_port(MW_GPIO::Port port);
    uint16_t get_hal_gpio_pin(MW_GPIO::Pin pin);
#endif
}  // namespace platform

#endif  // __MAPPER_HPP