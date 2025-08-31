#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include "middleware_interfaces.hpp"

namespace MW_GPIO {
    class TestGPIO : public IGPIO {
       public:
        void write_pin(Port port, Pin pin, State state) override {
            (void) port;
            (void) pin;
            (void) state;
            // TODO: Implement GPIO write logic
        }
        State read_pin(Port port, Pin pin) override {
            (void) port;
            (void) pin;
            // TODO: Implement GPIO read logic
            return State::LOW;
        }
        void toggle_pin(Port port, Pin pin) override {
            (void) port;
            (void) pin;
            // TODO: Implement GPIO toggle logic
        }
    };
}  // namespace MW_GPIO

namespace MW_TIM {
    class TestPWM : public IPWM {
       public:
        void start(Timer timer, Channel channel) override {
            (void) timer;
            (void) channel;
            // TODO: Implement PWM start logic
        }
        void stop(Timer timer, Channel channel) override {
            (void) timer;
            (void) channel;
            // TODO: Implement PWM stop logic
        }
        void set_duty_cycle(Timer timer, Channel channel,
                            uint32_t compare) override {
            (void) timer;
            (void) channel;
            (void) compare;
            // TODO: Implement PWM duty cycle logic
        }
    };
}  // namespace MW_TIM

namespace MW_CAN {
    class TestCAN : public ICAN {
       public:
        void send_data(BUS bus, uint32_t id, const std::array<uint8_t, 8>& data,
                       uint32_t length) override {
            (void) bus;
            (void) id;
            (void) data;
            (void) length;
            // TODO: Implement CAN send logic
        }
        bool receive_data(BUS bus, FIFO fifo, uint32_t& id, uint32_t& length,
                          std::array<uint8_t, 8>& data) override {
            (void) bus;
            (void) fifo;
            (void) id;
            (void) length;
            (void) data;
            // TODO: Implement CAN receive logic
            return false;
        }
        void start(BUS bus) override {
            (void) bus;
            // TODO: Implement CAN start logic
        }
        void stop(BUS bus) override {
            (void) bus;
            // TODO: Implement CAN stop logic
        }
        void activate_notification(BUS bus,
                                   Notification notification) override {
            (void) bus;
            (void) notification;
            // TODO: Implement CAN notification logic
        }
        void configure_filter(BUS bus, Filter filter_configuration) override {
            (void) bus;
            (void) filter_configuration;
            // TODO: Implement CAN filter configuration logic
        }
    };
}  // namespace MW_CAN

namespace MW_UART {
    class TestUART : public IUART {
       public:
        void send_data(Peripheral uart,
                       const std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                       uint32_t length, uint32_t timeout) override {
            (void) uart;
            (void) data;
            (void) length;
            (void) timeout;
            // TODO: Implement UART send logic
        }
        bool receive_data(Peripheral uart,
                          std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                          uint32_t& length) override {
            (void) uart;
            (void) data;
            (void) length;
            // TODO: Implement UART receive logic
            return false;
        }
        void abort_receive(Peripheral uart) override {
            (void) uart;
            // TODO: Implement UART abort receive logic
        }
        void abort_transmit(Peripheral uart) override {
            (void) uart;
            // TODO: Implement UART abort transmit logic
        }
    };
}  // namespace MW_UART

namespace MW_I2C {
    class TestI2C : public II2C {
       public:
        void master_transmit(Periperhal i2c, uint8_t device_address,
                             const uint8_t* data, size_t size,
                             uint32_t timeout) override {
            (void) i2c;
            (void) device_address;
            (void) data;
            (void) size;
            (void) timeout;
            // TODO: Implement I2C master transmit logic
        }
        void master_receive(Periperhal i2c, uint8_t device_address,
                            uint8_t* data, size_t size,
                            uint32_t timeout) override {
            (void) i2c;
            (void) device_address;
            (void) data;
            (void) size;
            (void) timeout;
            // TODO: Implement I2C master receive logic
        }
        void slave_transmit(Periperhal i2c, const uint8_t* data, size_t size,
                            uint32_t timeout) override {
            (void) i2c;
            (void) data;
            (void) size;
            (void) timeout;
            // TODO: Implement I2C slave transmit logic
        }
        void slave_receive(Periperhal i2c, uint8_t* data, size_t size,
                           uint32_t timeout) override {
            (void) i2c;
            (void) data;
            (void) size;
            (void) timeout;
            // TODO: Implement I2C slave receive logic
        }
    };
}  // namespace MW_I2C

namespace MW_RTOS {
    class TestRTOS : public IRTOS {
       private:
        TickType current_tick_ms = 0;

       public:
        void delay_until(uint32_t* previous_wake, uint32_t ms) override {
            current_tick_ms = *previous_wake + ms;
            *previous_wake += current_tick_ms;
        }

        void delay(uint32_t ms) override { current_tick_ms += ms; }

        TickType get_current_tick() override { return current_tick_ms; }

        bool queue_create(QueueHandle& queue, size_t queue_length,
                          size_t item_size) override {
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

        bool queue_overwrite(QueueHandle queue, void* data_ptr) override {
            if (!queue || !data_ptr || !queue->byte_queue) {
                return false;
            }

            if (queue->front_index == queue->back_index &&
                queue->item_count != 0) {
                queue->front_index =
                    (queue->front_index + 1) % queue->queue_length;
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

        bool queue_overwrite_from_isr(
            QueueHandle queue, void* data_ptr,
            bool* awaken_higher_prio = nullptr) override {
            if (awaken_higher_prio != nullptr) {
                *awaken_higher_prio = false;
            }

            if (!queue || !data_ptr || !queue->byte_queue) {
                return false;
            }

            if (queue->front_index == queue->back_index &&
                queue->item_count != 0) {
                queue->front_index =
                    (queue->front_index + 1) % queue->queue_length;
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

        bool queue_pushback(QueueHandle queue, void* data_ptr,
                            TickType ticks_to_wait = 0) override {
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

        bool queue_pushback_from_isr(
            QueueHandle queue, void* data_ptr,
            bool* awaken_higher_prio = nullptr) override {
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

        bool queue_peek(QueueHandle queue, void* data_ptr,
                        TickType ticks_to_wait = 0) override {
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

        bool queue_get(QueueHandle queue, void* data_ptr,
                       TickType ticks_to_wait = 0) override {
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
    };
}  // namespace MW_RTOS