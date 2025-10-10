#ifndef __MIDDLEWARE_MOCKS_HPP
#define __MIDDLEWARE_MOCKS_HPP

#include <gmock/gmock.h>
#include "middleware_interfaces.hpp"

class MockIGPIO : public MW_GPIO::IGPIO {
   public:
    MOCK_METHOD(void, write_pin,
                (MW_GPIO::Port port, MW_GPIO::Pin pin, MW_GPIO::State state),
                (override));
    MOCK_METHOD(MW_GPIO::State, read_pin,
                (MW_GPIO::Port port, MW_GPIO::Pin pin), (override));
    MOCK_METHOD(void, toggle_pin, (MW_GPIO::Port port, MW_GPIO::Pin pin),
                (override));
};

class MockIPWM : public MW_TIM::IPWM {
   public:
    MOCK_METHOD(void, start, (MW_TIM::Timer timer, MW_TIM::Channel channel),
                (override));
    MOCK_METHOD(void, stop, (MW_TIM::Timer timer, MW_TIM::Channel channel),
                (override));
    MOCK_METHOD(void, set_compare,
                (MW_TIM::Timer timer, MW_TIM::Channel channel,
                 uint32_t compare),
                (override));
    MOCK_METHOD(void, set_autoreload,
                (MW_TIM::Timer timer, uint32_t autoreload), (override));
    MOCK_METHOD(void, set_counter, (MW_TIM::Timer timer, uint32_t counter),
                (override));
};

class MockICAN : public MW_CAN::ICAN {
   public:
    MOCK_METHOD(bool, send_data,
                (MW_CAN::BUS bus, uint32_t id,
                 (const std::array<uint8_t, 8>& data), uint32_t length),
                (override));
    MOCK_METHOD(bool, receive_data,
                (MW_CAN::BUS bus, MW_CAN::FIFO fifo, uint32_t& id,
                 uint32_t& length, (std::array<uint8_t, 8> & data)),
                (override));
    MOCK_METHOD(bool, start, (MW_CAN::BUS bus), (override));
    MOCK_METHOD(bool, stop, (MW_CAN::BUS bus), (override));
    MOCK_METHOD(bool, activate_notification,
                (MW_CAN::BUS bus, MW_CAN::Notification notification),
                (override));
    MOCK_METHOD(bool, configure_filter,
                (MW_CAN::BUS bus, MW_CAN::Filter filter_configuration),
                (override));
};

class MockIUART : public MW_UART::IUART {
   public:
    MOCK_METHOD(void, send_data,
                (MW_UART::Peripheral uart, const uint8_t* data, uint32_t length,
                 uint32_t timeout),
                (override));
    MOCK_METHOD(bool, receive_data,
                (MW_UART::Peripheral uart, uint8_t* data, uint32_t& length),
                (override));
    MOCK_METHOD(void, abort_receive, (MW_UART::Peripheral uart), (override));
    MOCK_METHOD(void, abort_transmit, (MW_UART::Peripheral uart), (override));
};

class MockII2C : public MW_I2C::II2C {
   public:
    MOCK_METHOD(void, master_transmit,
                (MW_I2C::Periperhal i2c, uint8_t device_address,
                 const uint8_t* data, size_t size, uint32_t timeout),
                (override));
    MOCK_METHOD(void, master_receive,
                (MW_I2C::Periperhal i2c, uint8_t device_address, uint8_t* data,
                 size_t size, uint32_t timeout),
                (override));
    MOCK_METHOD(void, slave_transmit,
                (MW_I2C::Periperhal i2c, const uint8_t* data, size_t size,
                 uint32_t timeout),
                (override));
    MOCK_METHOD(void, slave_receive,
                (MW_I2C::Periperhal i2c, uint8_t* data, size_t size,
                 uint32_t timeout),
                (override));
};

class MockIRTOS : public MW_RTOS::IRTOS {
   public:
    MOCK_METHOD(void, delay_until, (uint32_t* previous_wake, uint32_t ms),
                (override));
    MOCK_METHOD(void, delay, (uint32_t ms), (override));
    MOCK_METHOD(MW_RTOS::TickType, get_current_tick, (), (override));
    MOCK_METHOD(MW_RTOS::TickType, ms_to_ticks, (uint32_t ms), (override));
    MOCK_METHOD(bool, queue_create,
                (MW_RTOS::QueueHandle & queue, size_t queue_length,
                 size_t item_size),
                (override));
    MOCK_METHOD(bool, queue_overwrite,
                (MW_RTOS::QueueHandle queue, void* data_ptr), (override));
    MOCK_METHOD(bool, queue_pushback,
                (MW_RTOS::QueueHandle queue, void* data_ptr,
                 MW_RTOS::TickType ticks_to_wait),
                (override));
    MOCK_METHOD(bool, queue_overwrite_from_isr,
                (MW_RTOS::QueueHandle queue, void* data_ptr,
                 bool* awaken_higher_prio),
                (override));
    MOCK_METHOD(bool, queue_pushback_from_isr,
                (MW_RTOS::QueueHandle queue, void* data_ptr,
                 bool* awaken_higher_prio),
                (override));
    MOCK_METHOD(bool, queue_peek,
                (MW_RTOS::QueueHandle queue, void* data_ptr,
                 MW_RTOS::TickType ticks_to_wait),
                (override));
    MOCK_METHOD(bool, queue_get,
                (MW_RTOS::QueueHandle queue, void* data_ptr,
                 MW_RTOS::TickType ticks_to_wait),
                (override));
};

#endif