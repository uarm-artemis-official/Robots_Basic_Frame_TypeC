#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include "middleware_classes.hpp"
#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace MW_BASE {
    bool TestBase::init() {
        return true;
    }

    void TestBase::delay_ms(uint32_t ms) {
        (void) ms;
    }
}  // namespace MW_BASE

namespace MW_GPIO {
    bool TestGPIO::init() {
        return true;
    }

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
    bool TestTIM::init() {
        return true;
    }

    bool TestTIM::base_start(Timer timer, BaseStartMode mode) {
        (void) timer;
        (void) mode;
        // TODO: Implement timer base start logic
        return true;
    }

    bool TestPWM::start(Timer timer, Channel channel) {
        (void) timer;
        (void) channel;
        // TODO: Implement PWM start logic
        return true;
    }
    bool TestPWM::init() {
        return true;
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

    void TestPWM::set_autoreload(Timer timer, uint32_t autoreload) {
        (void) timer;
        (void) autoreload;
        // TODO: Implement
    }

    void TestPWM::set_counter(Timer timer, uint32_t counter) {
        (void) timer;
        (void) counter;
        // TODO: Implement
    }
}  // namespace MW_TIM

namespace MW_CAN {
    bool TestCAN::init() {
        return true;
    }

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
    bool TestUART::init() {
        return true;
    }

    void TestUART::send_data(Peripheral uart, const uint8_t* data,
                             uint32_t length, uint32_t timeout) {
        (void) uart;
        (void) data;
        (void) length;
        (void) timeout;
        // TODO: Implement UART send logic
    }
    void TestUART::send_data(Peripheral uart, std::span<const std::byte> data,
                             uint32_t timeout) {
        (void) uart;
        (void) data;
        (void) timeout;
    }
    bool TestUART::receive_data(Peripheral uart, uint8_t* data,
                                uint32_t length) {
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

    void TestUART::clear_flags(Peripheral uart, uint32_t flags_to_clear) {
        (void) uart;
        (void) flags_to_clear;
        // TODO: Implement UART clear flag logic
    }
}  // namespace MW_UART

namespace MW_I2C {
    bool TestI2C::init() {
        return true;
    }

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

    void TestI2C::mem_write(Periperhal i2c, uint8_t device_address,
                            uint16_t memory_address,
                            uint16_t memory_address_size, const uint8_t* data,
                            size_t size, uint32_t timeout) {
        (void) i2c;
        (void) device_address;
        (void) memory_address;
        (void) memory_address_size;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C memory write logic
    }

    void TestI2C::mem_read(Periperhal i2c, uint8_t device_address,
                           uint16_t memory_address,
                           uint16_t memory_address_size, uint8_t* data,
                           size_t size, uint32_t timeout) {
        (void) i2c;
        (void) device_address;
        (void) memory_address;
        (void) memory_address_size;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement I2C memory read logic
    }
}  // namespace MW_I2C

namespace MW_SPI {
    bool TestSPI::init() {
        return true;
    }

    bool TestSPI::transmit(Peripheral spi, const uint8_t* data, size_t size,
                           uint32_t timeout) {
        (void) spi;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement SPI transmit logic
        return false;
    }

    bool TestSPI::receive(Peripheral spi, uint8_t* data, size_t size,
                          uint32_t timeout) {
        (void) spi;
        (void) data;
        (void) size;
        (void) timeout;
        // TODO: Implement SPI receive logic
        return false;
    }

    bool TestSPI::transmit_receive(Peripheral spi, const uint8_t* tx,
                                   uint8_t* rx, size_t size, uint32_t timeout) {
        (void) spi;
        (void) tx;
        (void) rx;
        (void) size;
        (void) timeout;
        // TODO: Implement SPI transmit/receive logic
        return false;
    }
}  // namespace MW_SPI

namespace MW_RTOS {
    bool TestRTOS::init() {
        return true;
    }

    bool TestRTOS::task_create(TaskHandle& task, const char* task_name,
                               TaskRoutine task_routine, void* task_arg,
                               size_t stack_depth, TaskPriority priority) {
        ASSERT(task_name != nullptr, "Task name cannot be nullptr.");
        ASSERT(task_routine != nullptr, "Task routine cannot be nullptr.");

        task = static_cast<TaskHandle>(malloc(sizeof(MockRTOSTask)));
        if (task == nullptr) {
            return false;
        }

        task->name = task_name;
        task->routine = task_routine;
        task->argument = task_arg;
        task->stack_depth = stack_depth;
        task->priority = priority;
        return true;
    }

    void TestRTOS::critical_section_enter() {
        // No-op in test middleware.
    }

    void TestRTOS::critical_section_exit() {
        // No-op in test middleware.
    }

    bool TestRTOS::event_group_create(EventGroupHandle& event_group) {
        event_group =
            static_cast<EventGroupHandle>(malloc(sizeof(MockRTOSEventGroup)));
        if (event_group == nullptr) {
            return false;
        }
        event_group->bits = 0;
        return true;
    }

    EventBits TestRTOS::event_group_set_bits(EventGroupHandle event_group,
                                             EventBits bits_to_set) {
        if (event_group == nullptr) {
            return 0;
        }
        event_group->bits |= bits_to_set;
        return event_group->bits;
    }

    EventBits TestRTOS::event_group_clear_bits(EventGroupHandle event_group,
                                               EventBits bits_to_clear) {
        if (event_group == nullptr) {
            return 0;
        }
        event_group->bits &= ~bits_to_clear;
        return event_group->bits;
    }

    EventBits TestRTOS::event_group_wait_bits(EventGroupHandle event_group,
                                              EventBits bits_to_wait_for,
                                              bool clear_on_exit,
                                              bool wait_for_all_bits,
                                              uint32_t ticks_to_wait) {
        (void) ticks_to_wait;
        if (event_group == nullptr) {
            return 0;
        }

        const EventBits current = event_group->bits;
        const bool condition_met =
            wait_for_all_bits
                ? ((current & bits_to_wait_for) == bits_to_wait_for)
                : ((current & bits_to_wait_for) != 0);

        if (condition_met && clear_on_exit) {
            event_group->bits &= ~bits_to_wait_for;
        }

        return current;
    }

    EventBits TestRTOS::event_group_sync(EventGroupHandle event_group,
                                         EventBits bits_to_set,
                                         EventBits bits_to_wait_for,
                                         uint32_t ticks_to_wait) {
        (void) bits_to_wait_for;
        (void) ticks_to_wait;
        if (event_group == nullptr) {
            return 0;
        }

        event_group->bits |= bits_to_set;
        return event_group->bits;
    }

    void TestRTOS::delay_until_ms(uint32_t* previous_wake, uint32_t ms) {
        current_tick_ms = *previous_wake + ms;
        *previous_wake += current_tick_ms;
    }

    void TestRTOS::delay_ms(uint32_t ms) {
        current_tick_ms += ms;
    }

    void TestRTOS::delay_until_us(uint32_t* previous_wake, uint32_t us) {
        const uint32_t converted_ms = (us + 999U) / 1000U;
        delay_until_ms(previous_wake, converted_ms);
    }

    void TestRTOS::delay_us(uint32_t us) {
        const uint32_t converted_ms = (us + 999U) / 1000U;
        delay_ms(converted_ms);
    }

    TickType TestRTOS::get_current_tick() {
        return current_tick_ms;
    }

    TickType TestRTOS::ms_to_ticks(uint32_t ms) {
        return static_cast<TickType>(ms);
    }

    bool TestRTOS::queue_create(QueueHandle& queue, size_t queue_length,
                                size_t item_size) {
        constexpr size_t MAX_QUEUE_SIZE = 5000;
        ASSERT(queue_length * item_size > 0,
               "Queue length and item size must be greater than zero");
        ASSERT(queue_length * item_size <= MAX_QUEUE_SIZE,
               "Requested queue size too large for test environment");

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

    bool TestRTOS::timer_create(TimerHandle& timer, const char* timer_name,
                                uint32_t duration_ms, TimerMode mode,
                                TimerCallback callback) {
        ASSERT(timer_name != nullptr, "Timer name cannot be nullptr.");
        ASSERT(callback != nullptr, "Timer callback cannot be nullptr.");

        timer = static_cast<TimerHandle>(malloc(sizeof(MockRTOSTimer)));
        if (timer == nullptr) {
            return false;
        }

        timer->name = timer_name;
        timer->period_ms = duration_ms;
        timer->mode = mode;
        timer->is_active = false;
        return true;
    }

    bool TestRTOS::timer_start(TimerHandle timer) {
        if (timer == nullptr) {
            return false;
        }
        timer->is_active = true;
        return true;
    }

    bool TestRTOS::timer_start_from_isr(TimerHandle timer,
                                        bool* awaken_higher_prio) {
        if (awaken_higher_prio != nullptr) {
            *awaken_higher_prio = false;
        }
        return timer_start(timer);
    }

    bool TestRTOS::timer_reset(TimerHandle timer) {
        if (timer == nullptr) {
            return false;
        }
        timer->is_active = true;
        return true;
    }

    bool TestRTOS::timer_stop(TimerHandle timer) {
        if (timer == nullptr) {
            return false;
        }
        timer->is_active = false;
        return true;
    }

    bool TestRTOS::timer_stop_from_isr(TimerHandle timer,
                                       bool* awaken_higher_prio) {
        if (awaken_higher_prio != nullptr) {
            *awaken_higher_prio = false;
        }
        return timer_stop(timer);
    }

    bool TestRTOS::timer_delete(TimerHandle timer) {
        if (timer == nullptr) {
            return false;
        }

        free(timer);
        return true;
    }
}  // namespace MW_RTOS