#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include "middleware_classes.hpp"
#include "middleware_interfaces.hpp"

namespace MW_GPIO {
    void TestGPIO::write_pin(Port port, Pin pin, State state) {
        (void) port;
        (void) pin;
        (void) state;
        // TODO: Implement GPIO write logic
    }
    State TestGPIO::read_pin(Port port, Pin pin) {
        (void) port;
        (void) pin;
        // TODO: Implement GPIO read logic
        return State::LOW;
    }
    void TestGPIO::toggle_pin(Port port, Pin pin) {
        (void) port;
        (void) pin;
        // TODO: Implement GPIO toggle logic
    }
}  // namespace MW_GPIO

namespace MW_TIM {
    void TestPWM::start(Timer timer, Channel channel) {
        (void) timer;
        (void) channel;
        // TODO: Implement PWM start logic
    }
    void TestPWM::stop(Timer timer, Channel channel) {
        (void) timer;
        (void) channel;
        // TODO: Implement PWM stop logic
    }
    void TestPWM::set_compare(Timer timer, Channel channel, uint32_t compare) {
        (void) timer;
        (void) channel;
        (void) compare;
        // TODO: Implement PWM duty cycle logic
    }

    void set_autoreload(Timer timer, uint32_t autoreload) {
        (void) timer;
        (void) autoreload;
        // TODO: Implement
    }

    void set_counter(Timer timer, uint32_t counter) {
        (void) timer;
        (void) counter;
        // TODO: Implement
    }
}  // namespace MW_TIM

namespace MW_CAN {
    bool TestCAN::send_data(BUS bus, uint32_t id, uint32_t ext_id,
                            const uint8_t* data, uint32_t length) {
        (void) bus;
        (void) id;
        (void) ext_id;
        (void) data;
        (void) length;
        // TODO: Implement CAN send logic
        return false;
    }
    bool TestCAN::receive_data(BUS bus, FIFO fifo, uint32_t& id,
                               uint32_t& ext_id, uint8_t* dst,
                               uint32_t& length) {
        (void) bus;
        (void) fifo;
        (void) id;
        (void) ext_id;
        (void) length;
        (void) dst;
        // TODO: Implement CAN receive logic
        return false;
    }
    bool TestCAN::start(BUS bus) {
        (void) bus;
        // TODO: Implement CAN start logic
        return false;
    }
    bool TestCAN::stop(BUS bus) {
        (void) bus;
        // TODO: Implement CAN stop logic
        return false;
    }
    bool TestCAN::activate_notification(BUS bus, Notification notification) {
        (void) bus;
        (void) notification;
        // TODO: Implement CAN notification logic
        return false;
    }
    bool TestCAN::configure_filter(BUS bus, Filter filter_configuration) {
        (void) bus;
        (void) filter_configuration;
        // TODO: Implement CAN filter configuration logic
        return false;
    }
}  // namespace MW_CAN

namespace MW_UART {
    void TestUART::send_data(Peripheral uart, const uint8_t* data,
                             uint32_t length, uint32_t timeout) {
        (void) uart;
        (void) data;
        (void) length;
        (void) timeout;
        // TODO: Implement UART send logic
    }
    bool TestUART::receive_data(Peripheral uart, uint8_t* data,
                                uint32_t& length) {
        (void) uart;
        (void) data;
        (void) length;
        // TODO: Implement UART receive logic
        return false;
    }
    void TestUART::abort_receive(Peripheral uart) {
        (void) uart;
        // TODO: Implement UART abort receive logic
    }
    void TestUART::abort_transmit(Peripheral uart) {
        (void) uart;
        // TODO: Implement UART abort transmit logic
    }
}  // namespace MW_UART

namespace MW_I2C {
    void TestI2C::master_transmit(Periperhal i2c, uint8_t device_address,
                                  const uint8_t* data, size_t size,
                                  uint32_t timeout) {
        (void) i2c;
        (void) device_address;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C master transmit logic
    }
    void TestI2C::master_receive(Periperhal i2c, uint8_t device_address,
                                 uint8_t* data, size_t size, uint32_t timeout) {
        (void) i2c;
        (void) device_address;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C master receive logic
    }
    void TestI2C::slave_transmit(Periperhal i2c, const uint8_t* data,
                                 size_t size, uint32_t timeout) {
        (void) i2c;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C slave transmit logic
    }
    void TestI2C::slave_receive(Periperhal i2c, uint8_t* data, size_t size,
                                uint32_t timeout) {
        (void) i2c;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C slave receive logic
    }
}  // namespace MW_I2C

namespace MW_RTOS {
    void TestRTOS::delay_until(uint32_t* previous_wake, uint32_t ms) {
        current_tick_ms = *previous_wake + ms;
        *previous_wake += current_tick_ms;
    }

    void TestRTOS::delay(uint32_t ms) {
        current_tick_ms += ms;
    }

    TickType TestRTOS::get_current_tick() {
        return current_tick_ms;
    }

    TickType TestRTOS::ms_to_ticks(uint32_t ms) {
        return static_cast<TickType>(ms);
    }

    bool TestRTOS::queue_create(QueueHandle& queue, size_t queue_length,
                                size_t item_size) {
        queue = static_cast<QueueHandle>(malloc(sizeof(MockRTOSQueue)));
        if (queue == nullptr) {
            return false;
        }
        queue->queue_length = queue_length;
        queue->item_size = item_size;
        queue->item_count = 0;
        queue->front_index = 0;
        queue->back_index = 0;
        queue->byte_queue =
            static_cast<uint8_t*>(malloc(queue_length * item_size));
        if (queue->byte_queue == nullptr) {
            free(queue);
            queue = nullptr;
            return false;
        } else {
            memset(queue->byte_queue, 0, queue_length * item_size);
        }
        return true;
    }

    bool TestRTOS::queue_overwrite(QueueHandle queue, void* data_ptr) {
        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        if (queue->front_index == queue->back_index && queue->item_count != 0) {
            queue->front_index = (queue->front_index + 1) % queue->queue_length;
        }
        // Copy data_ptr to the back of the queue
        size_t slot = queue->back_index % queue->queue_length;
        uint8_t* dest = queue->byte_queue + slot * queue->item_size;
        std::memcpy(dest, data_ptr, queue->item_size);
        queue->back_index = (queue->back_index + 1) % queue->queue_length;
        queue->item_count =
            std::min(queue->item_count + 1, queue->queue_length);
        return true;
    }

    bool TestRTOS::queue_overwrite_from_isr(QueueHandle queue, void* data_ptr,
                                            bool* awaken_higher_prio) {
        if (awaken_higher_prio != nullptr) {
            *awaken_higher_prio = false;
        }

        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        if (queue->front_index == queue->back_index && queue->item_count != 0) {
            queue->front_index = (queue->front_index + 1) % queue->queue_length;
        }
        // Copy data_ptr to the back of the queue
        size_t slot = queue->back_index % queue->queue_length;
        uint8_t* dest = queue->byte_queue + slot * queue->item_size;
        std::memcpy(dest, data_ptr, queue->item_size);
        queue->back_index = (queue->back_index + 1) % queue->queue_length;
        queue->item_count =
            std::min(queue->item_count + 1, queue->queue_length);
        return true;
    }

    bool TestRTOS::queue_pushback(QueueHandle queue, void* data_ptr,
                                  TickType ticks_to_wait) {
        (void) ticks_to_wait;
        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        // Check if queue is full
        if (queue->item_count >= queue->queue_length) {
            return false;
        }

        // Copy data_ptr to the back of the queue
        size_t slot = queue->back_index % queue->queue_length;
        uint8_t* dest = queue->byte_queue + slot * queue->item_size;
        std::memcpy(dest, data_ptr, queue->item_size);
        queue->back_index = (queue->back_index + 1) % queue->queue_length;
        queue->item_count++;
        return true;
    }

    bool TestRTOS::queue_pushback_from_isr(QueueHandle queue, void* data_ptr,
                                           bool* awaken_higher_prio) {
        if (awaken_higher_prio != nullptr) {
            *awaken_higher_prio = false;
        }

        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        // Check if queue is full
        if (queue->item_count >= queue->queue_length) {
            return false;
        }
        // Copy data_ptr to the back of the queue
        size_t slot = queue->back_index % queue->queue_length;
        uint8_t* dest = queue->byte_queue + slot * queue->item_size;
        std::memcpy(dest, data_ptr, queue->item_size);
        queue->back_index = (queue->back_index + 1) % queue->queue_length;
        queue->item_count++;
        return true;
    }

    bool TestRTOS::queue_peek(QueueHandle queue, void* data_ptr,
                              TickType ticks_to_wait) {
        (void) ticks_to_wait;
        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        // Check if queue is empty
        if (queue->item_count == 0) {
            return false;
        }

        // Copy the front item to data_ptr, but do not modify indices or item_count
        size_t slot = queue->front_index % queue->queue_length;
        uint8_t* src = queue->byte_queue + slot * queue->item_size;
        std::memcpy(data_ptr, src, queue->item_size);
        return true;
    }

    bool TestRTOS::queue_get(QueueHandle queue, void* data_ptr,
                             TickType ticks_to_wait) {
        (void) ticks_to_wait;
        if (!queue || !data_ptr || !queue->byte_queue) {
            return false;
        }

        // Check if queue is empty
        if (queue->item_count == 0) {
            return false;
        }

        // Copy the front item to data_ptr
        size_t slot = queue->front_index % queue->queue_length;
        uint8_t* src = queue->byte_queue + slot * queue->item_size;
        std::memcpy(data_ptr, src, queue->item_size);
        queue->front_index = (queue->front_index + 1) % queue->queue_length;
        queue->item_count--;
        return true;
    }
}  // namespace MW_RTOS