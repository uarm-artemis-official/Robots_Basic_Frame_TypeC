#ifndef __MIDDLEWARE_CLASSES_HPP
#define __MIDDLEWARE_CLASSES_HPP

#include "middleware_interfaces.hpp"

namespace MW_GPIO {
    class TestGPIO : public IGPIO {
       public:
        void write_pin(Port port, Pin pin, State state) override;
        State read_pin(Port port, Pin pin) override;
        void toggle_pin(Port port, Pin pin) override;
    };

    /**
     * @brief Implementation of IGPIO interface for DJI's Robomaster Type-C development boards.
     * GPIO pins are primarily used for toggling LEDs.
     */
    class GPIO : public IGPIO {
       public:
        void write_pin(Port port, Pin pin, State state) override;
        State read_pin(Port port, Pin pin) override;
        void toggle_pin(Port port, Pin pin) override;
    };
}  // namespace MW_GPIO

namespace MW_TIM {
    class TestPWM : public IPWM {
       public:
        void start(Timer timer, Channel channel) override;
        void stop(Timer timer, Channel channel) override;
        void set_compare(Timer timer, Channel channel,
                         uint32_t compare) override;
        void set_autoreload(Timer timer, uint32_t autoreload) override;
        void set_counter(Timer timer, uint32_t counter) override;
    };

    /**
     * Implementation of IPWM for DJI's Robomaster Type C development board.
     */
    class PWM : public IPWM {
       public:
        void start(Timer timer, Channel channel) override;
        void stop(Timer timer, Channel channel) override;
        void set_compare(Timer timer, Channel channel,
                         uint32_t compare) override;
        void set_autoreload(Timer timer, uint32_t autoreload) override;
        void set_counter(Timer timer, uint32_t counter) override;
    };
}  // namespace MW_TIM

namespace MW_CAN {
    class TestCAN : public ICAN {
       public:
        bool send_data(BUS bus, uint32_t id, uint32_t ext_id,
                       const uint8_t* data, uint32_t length) override;
        bool receive_data(BUS bus, FIFO fifo, uint32_t& id, uint32_t& ext_id,
                          uint8_t* dst, uint32_t& length) override;
        bool start(BUS bus) override;
        bool stop(BUS bus) override;
        bool activate_notification(BUS bus, Notification notification) override;
        bool configure_filter(BUS bus, Filter filter_configuration) override;
    };

    /**
     * @brief Implementation of ICAN interface for utilizing CAN buses on DJI's Type-C development board.
     * This class provides methods to send and receive CAN messages, start and stop the CAN bus,
     * activate notifications, and configure filters.
     */
    class CAN : public ICAN {
       public:
        bool send_data(BUS bus, uint32_t id, uint32_t ext_id,
                       const uint8_t* data, uint32_t length) override;
        bool receive_data(BUS bus, FIFO fifo, uint32_t& id, uint32_t& ext_id,
                          uint8_t* dst, uint32_t& length) override;
        bool start(BUS bus) override;
        bool stop(BUS bus) override;
        bool activate_notification(BUS bus, Notification notification) override;
        bool configure_filter(BUS bus, Filter filter_configuration) override;
    };
}  // namespace MW_CAN

namespace MW_UART {
    class TestUART : public IUART {
       public:
        void send_data(Peripheral uart, const uint8_t* data, uint32_t length,
                       uint32_t timeout) override;
        void send_data(Peripheral uart, std::span<const std::byte> data,
                       uint32_t timeout) override;
        bool receive_data(Peripheral uart, uint8_t* data,
                          uint32_t length) override;
        void abort_receive(Peripheral uart) override;
        void abort_transmit(Peripheral uart) override;
        void clear_flags(Peripheral uart, uint32_t flags_to_clear) override;
    };

    /**
     * @brief Implementation of IUART interface for utilizing UART on DJI's Type-C development board.
     * This class provides methods to send and receive UART messages, and abort reception and transmissions
     * of UART messages.
     */
    class UART : public IUART {
       public:
        void send_data(Peripheral uart, const uint8_t* data, uint32_t length,
                       uint32_t timeout) override;
        void send_data(Peripheral uart, std::span<const std::byte> data,
                       uint32_t timeout) override;
        bool receive_data(Peripheral uart, uint8_t* data,
                          uint32_t length) override;
        void abort_receive(Peripheral uart) override;
        void abort_transmit(Peripheral uart) override;
        void clear_flags(Peripheral uart, uint32_t flags_to_clear) override;
    };
}  // namespace MW_UART

namespace MW_I2C {
    class TestI2C : public II2C {
       public:
        void master_transmit(Periperhal i2c, uint8_t device_address,
                             const uint8_t* data, size_t size,
                             uint32_t timeout) override;
        void master_receive(Periperhal i2c, uint8_t device_address,
                            uint8_t* data, size_t size,
                            uint32_t timeout) override;
        void slave_transmit(Periperhal i2c, const uint8_t* data, size_t size,
                            uint32_t timeout) override;
        void slave_receive(Periperhal i2c, uint8_t* data, size_t size,
                           uint32_t timeout) override;
        void mem_write(Periperhal i2c, uint8_t device_address,
                       uint16_t memory_address, uint16_t memory_address_size,
                       const uint8_t* data, size_t size,
                       uint32_t timeout) override;
        void mem_read(Periperhal i2c, uint8_t device_address,
                      uint16_t memory_address, uint16_t memory_address_size,
                      uint8_t* data, size_t size, uint32_t timeout) override;
    };

    class I2C : public II2C {
       public:
        void master_transmit(Periperhal i2c, uint8_t device_address,
                             const uint8_t* data, size_t size,
                             uint32_t timeout) override;
        void master_receive(Periperhal i2c, uint8_t device_address,
                            uint8_t* data, size_t size,
                            uint32_t timeout) override;
        void slave_transmit(Periperhal i2c, const uint8_t* data, size_t size,
                            uint32_t timeout) override;
        void slave_receive(Periperhal i2c, uint8_t* data, size_t size,
                           uint32_t timeout) override;
        void mem_write(Periperhal i2c, uint8_t device_address,
                       uint16_t memory_address, uint16_t memory_address_size,
                       const uint8_t* data, size_t size,
                       uint32_t timeout) override;
        void mem_read(Periperhal i2c, uint8_t device_address,
                      uint16_t memory_address, uint16_t memory_address_size,
                      uint8_t* data, size_t size, uint32_t timeout) override;
    };
}  // namespace MW_I2C

namespace MW_RTOS {
    class TestRTOS : public IRTOS {
       private:
        TickType current_tick_ms = 0;

       public:
        void delay_until(uint32_t* previous_wake, uint32_t ms) override;
        void delay(uint32_t ms) override;
        TickType get_current_tick() override;
        TickType ms_to_ticks(uint32_t ms) override;

        bool queue_create(QueueHandle& queue, size_t queue_length,
                          size_t item_size) override;
        bool queue_overwrite(QueueHandle queue, void* data_ptr) override;
        bool queue_pushback(QueueHandle queue, void* data_ptr,
                            TickType ticks_to_wait = 0) override;
        bool queue_overwrite_from_isr(
            QueueHandle queue, void* data_ptr,
            bool* awaken_higher_prio = nullptr) override;
        bool queue_pushback_from_isr(
            QueueHandle queue, void* data_ptr,
            bool* awaken_higher_prio = nullptr) override;
        bool queue_peek(QueueHandle queue, void* data_ptr,
                        TickType ticks_to_wait = 0) override;
        bool queue_get(QueueHandle queue, void* data_ptr,
                       TickType ticks_to_wait = 0) override;
    };

    class RTOS : public IRTOS {
       public:
        void delay_until(uint32_t* previous_wake, uint32_t ms) override;
        void delay(uint32_t ms) override;
        TickType get_current_tick() override;
        TickType ms_to_ticks(uint32_t ms) override;

        bool queue_create(QueueHandle& queue, size_t queue_length,
                          size_t item_size) override;
        bool queue_overwrite(QueueHandle queue, void* data_ptr) override;
        bool queue_pushback(QueueHandle queue, void* data_ptr,
                            TickType ticks_to_wait) override;
        bool queue_overwrite_from_isr(QueueHandle queue, void* data_ptr,
                                      bool* awakenHigherPrio) override;
        bool queue_pushback_from_isr(QueueHandle queue, void* data_ptr,
                                     bool* awakenHigherPrio) override;
        bool queue_peek(QueueHandle queue, void* data_ptr,
                        TickType ticks_to_wait) override;
        bool queue_get(QueueHandle queue, void* data_ptr,
                       TickType ticks_to_wait) override;
    };
}  // namespace MW_RTOS

#endif