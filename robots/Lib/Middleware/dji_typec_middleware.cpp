#include "can.h"
#include "middleware_classes.hpp"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "uarm_lib.hpp"
#include "usart.h"

namespace MW_GPIO {
    // Mapping functions between enum classes and HAL types.
    // ============================================================
    /**
     * @brief Get the GPIO port from the Port enum.
     * @param port The Port enum value.
     * @return The corresponding GPIO_TypeDef pointer.
     */
    GPIO_TypeDef* get_hal_port(Port port) {
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
    }

    /**
     * @brief Get the GPIO pin number from the Pin enum.
     * @param pin The Pin enum value.
     * @return The corresponding GPIO pin number value.
     */
    uint16_t get_hal_pin(Pin pin) {
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

    /**
     * @brief Convert the State enum to the corresponding HAL GPIO_PinState.
     * @param state The State enum value.
     * @return The corresponding GPIO_PinState value.
     */
    GPIO_PinState get_hal_state(State state) {
        switch (state) {
            case State::LOW:
                return GPIO_PIN_RESET;
            case State::HIGH:
                return GPIO_PIN_SET;
            default:
                ASSERT(false, "Unsupported GPIO pin state.");
        }
    }

    /**
     * @brief Convert the HAL GPIO_PinState to the State enum.
     * @param hal_state The HAL GPIO_PinState value.
     * @return The corresponding State enum value.
     */
    State get_state_from_hal(GPIO_PinState hal_state) {
        switch (hal_state) {
            case GPIO_PIN_RESET:
                return State::LOW;
            case GPIO_PIN_SET:
                return State::HIGH;
            default:
                ASSERT(false, "Unsupported HAL pin state.");
        }
    }

    /**
     * @brief Write a value to a GPIO pin.
     * @param port The GPIO port.
     * @param pin The GPIO pin.
     * @param state The desired state (HIGH or LOW) to write to the pin.
     */
    void GPIO::write_pin(Port port, Pin pin, State state) {
        HAL_GPIO_WritePin(get_hal_port(port), get_hal_pin(pin),
                          get_hal_state(state));
    }

    /**
     * @brief Read the value of a GPIO pin.
     * @param port The GPIO port.
     * @param pin The GPIO pin.
     * @return The current state (HIGH or LOW) of the pin.
     */
    State GPIO::read_pin(Port port, Pin pin) {
        return get_state_from_hal(
            HAL_GPIO_ReadPin(get_hal_port(port), get_hal_pin(pin)));
    }

    /**
     * @brief Toggle the state of a GPIO pin.
     * @param port The GPIO port.
     * @param pin The GPIO pin.
     */
    void GPIO::toggle_pin(Port port, Pin pin) {
        HAL_GPIO_TogglePin(get_hal_port(port), get_hal_pin(pin));
    }
}  // namespace MW_GPIO

namespace MW_TIM {
    /**
     * @brief Get HAL Timer handle for a Timer enum.
     * @param[in] timer Timer enum to get.
     * @return HAL Timer handle.
     */
    TIM_HandleTypeDef* get_hal_tim_handle(Timer timer) {
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
    }

    /**
     * @brief Get HAL timer channel for a Channel enum.
     * @param[in] channel Channel enum to get.
     * @return HAL timer channel value.
     */
    uint32_t get_hal_tim_channel(Channel channel) {
        switch (channel) {
            case Channel::CHANNEL_1:
                return TIM_CHANNEL_1;
            case Channel::CHANNEL_2:
                return TIM_CHANNEL_2;
            case Channel::CHANNEL_3:
                return TIM_CHANNEL_3;
            case Channel::CHANNEL_4:
                return TIM_CHANNEL_4;
            default:
                ASSERT(false, "Trying to get unsupported Timer channel.");
        }
    }
    /**
     * @brief Start PWM generation for a specific timer and channel.
     * @param[in] timer Timer to start.
     * @param[in] channel Channel of timer to start.
     */
    void PWM::start(Timer timer, Channel channel) {
        HAL_TIM_PWM_Start(get_hal_tim_handle(timer),
                          get_hal_tim_channel(channel));
    }

    /**
     * @brief Stop PWM generation for a specific timer and channel.
     * @param[in] timer Timer to stop.
     * @param[in] channel Channel of timer to stop.
     */
    void PWM::stop(Timer timer, Channel channel) {
        HAL_TIM_PWM_Stop(get_hal_tim_handle(timer),
                         get_hal_tim_channel(channel));
    }

    /**
     * @brief Set the compare value of a PWM signal generated by a timer.
     * Changing the compare value directly affects the duty cycle of the PWM
     * signal generated. To calculate the exact compare value for a desired
     * duty cycle use this formula:
     * \f$\frac{compare_value}{TIMx->ARR + 1} * 100%\f$
     * @param[in] timer Timer to set.
     * @param[in] channel Channel of timer to set.
     * @param[in] compare_value The compare value for the PWM signal.
     */
    void PWM::set_compare(Timer timer, Channel channel, uint32_t compare) {
        __HAL_TIM_SET_COMPARE(get_hal_tim_handle(timer),
                              get_hal_tim_channel(channel), compare);
    }

    void PWM::set_autoreload(Timer timer, uint32_t autoreload) {
        __HAL_TIM_SET_AUTORELOAD(get_hal_tim_handle(timer), autoreload);
    }

    void PWM::set_counter(Timer timer, uint32_t counter) {
        __HAL_TIM_SET_COUNTER(get_hal_tim_handle(timer), counter);
    }
}  // namespace MW_TIM

namespace MW_CAN {
    /**
     * @brief Get the HAL CAN handle from the BUS enum.
     * @param bus The BUS enum value.
     * @return The corresponding CAN_HandleTypeDef pointer.
     */
    CAN_HandleTypeDef* get_hal_can_handle(BUS bus) {
        switch (bus) {
            case BUS::CAN_1:
                return &hcan1;
            case BUS::CAN_2:
                return &hcan2;
            default:
                ASSERT(false, "Unsupported CAN bus");
        }
    }

    /**
     * @brief Get the HAL FIFO value from the FIFO enum.
     * @param fifo The FIFO enum value.
     * @return The corresponding HAL FIFO value.
     */
    uint32_t get_hal_fifo(FIFO fifo) {
        switch (fifo) {
            case FIFO::FIFO_0:
                return CAN_RX_FIFO0;
            case FIFO::FIFO_1:
                return CAN_RX_FIFO1;
            default:
                ASSERT(false, "Unsupported CAN FIFO");
        }
    }
    /**
     * @brief Send data over the CAN bus.
     * @pre 0 < length <= 8.
     * @param[in] bus The CAN bus to send data on.
     * @param[in] id The ID of the message to send.
     * @param[in] data The data to send.
     * @param[in] length The length of the data.
     */
    bool CAN::send_data(BUS bus, uint32_t id,
                        const std::array<uint8_t, 8>& data, uint32_t length) {
        ASSERT(0 < length && length <= 8,
               "0 bytes < CAN data length <= 8 bytes.");
        CAN_TxHeaderTypeDef tx_header;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = length;
        tx_header.StdId = id;
        return HAL_CAN_AddTxMessage(get_hal_can_handle(bus), &tx_header,
                                    data.data(), nullptr) == HAL_OK;
    }

    /**
     * @brief Receive data from the CAN bus.
     * @param[in] bus The CAN bus to receive data from.
     * @param[in] fifo The FIFO to read from.
     * @param[out] id The ID of the received message.
     * @param[out] length The length of the received data.
     * @param[out] data The buffer to store the received data.
     */
    bool CAN::receive_data(BUS bus, FIFO fifo, uint32_t& id, uint32_t& length,
                           std::array<uint8_t, 8>& data) {
        CAN_RxHeaderTypeDef rx_header;
        if (HAL_CAN_GetRxMessage(get_hal_can_handle(bus), get_hal_fifo(fifo),
                                 &rx_header, data.data()) != HAL_OK) {
            return false;  // Error in receiving message
        } else {
            id = rx_header.StdId;
            length = rx_header.DLC;
            return true;
        }
    }

    /**
     * @brief Start the CAN bus communication.
     * @param[in] bus The CAN bus to start.
     */
    bool CAN::start(BUS bus) {
        return HAL_CAN_Start(get_hal_can_handle(bus)) == HAL_OK;
    }

    /**
     * @brief Stop the CAN bus communication.
     * @param[in] bus The CAN bus to stop.
     */
    bool CAN::stop(BUS bus) {
        return HAL_CAN_Stop(get_hal_can_handle(bus)) == HAL_OK;
    }

    /**
     * @brief Activate a CAN notification.
     * @param[in] bus The CAN bus to activate the notification on.
     * @param[in] notification The notification type to activate.
     */
    bool CAN::activate_notification(BUS bus, Notification notification) {
        switch (notification) {
            case Notification::RX_FIFO0_MSG_PENDING:
                return HAL_CAN_ActivateNotification(
                           get_hal_can_handle(bus),
                           CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
            default:
                ASSERT(false, "Unsupported CAN notification");
        }
    }

    /**
     * @brief Configure a CAN filter.
     * @param[in] bus The CAN bus to configure.
     * @param[in] filter_configuration The filter configuration to apply.
     */
    bool CAN::configure_filter(BUS bus, Filter filter_configuration) {
        CAN_FilterTypeDef filter_config;
        filter_config.FilterIdHigh = filter_configuration.id_high;
        filter_config.FilterIdLow = filter_configuration.id_low;
        filter_config.FilterMaskIdHigh = filter_configuration.mask_id_high;
        filter_config.FilterMaskIdLow = filter_configuration.mask_id_low;
        filter_config.FilterFIFOAssignment =
            get_hal_fifo(filter_configuration.fifo_assignment);
        filter_config.FilterBank = filter_configuration.filter_bank;
        filter_config.SlaveStartFilterBank =
            filter_configuration.slave_start_filter_bank;
        filter_config.FilterMode =
            (filter_configuration.mode == FilterMode::IDMask)
                ? CAN_FILTERMODE_IDMASK
                : CAN_FILTERMODE_IDLIST;
        filter_config.FilterActivation =
            (filter_configuration.is_activated) ? ENABLE : DISABLE;
        return HAL_CAN_ConfigFilter(get_hal_can_handle(bus), &filter_config) ==
               HAL_OK;
    }
}  // namespace MW_CAN

namespace MW_UART {
    /**
     * @brief Get the HAL UART handle from the Peripheral enum.
     * @param uart The Peripheral enum value.
     * @return The corresponding UART_HandleTypeDef pointer.
     */
    UART_HandleTypeDef* get_hal_uart_handle(Peripheral uart) {
        switch (uart) {
            case Peripheral::UART1:
                return &huart1;
            case Peripheral::UART3:
                return &huart3;
            case Peripheral::UART6:
                return &huart6;
            default:
                ASSERT(false, "Unsupported UART peripheral");
        }
    }

    /**
     * @brief Send data over UART.
     * @pre 0 < length <= MAX_UART_BUFFER_SIZE.
     * @param[in] uart The UART peripheral to send data on.
     * @param[in] data The data to send.
     * @param[in] length The length of the data.
     * @param[in] timeout The timeout for the transmission.
     */
    void UART::send_data(Peripheral uart,
                         const std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                         uint32_t length, uint32_t timeout) {
        ASSERT(0 < length && length <= MAX_UART_BUFFER_SIZE,
               "UART data length must be > 0 and <= MAX_UART_BUFFER_SIZE");
        HAL_UART_Transmit(get_hal_uart_handle(uart), data.data(), length,
                          timeout);
    }

    /**
     * @brief Start reception of data from UART via DMA.
     * @pre 0 < length <= MAX_UART_BUFFER_SIZE
     * @param[in] uart The UART peripheral to receive data from.
     * @param[out] data The buffer to store the received data.
     * @param[out] length The length of the received data.
     * @return true if data was received successfully, false otherwise.
     */
    bool UART::receive_data(Peripheral uart,
                            std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                            uint32_t& length) {
        ASSERT(0 < length && length <= MAX_UART_BUFFER_SIZE,
               "UART data length must be > 0 and <= MAX_UART_BUFFER_SIZE");
        HAL_UART_Receive_DMA(get_hal_uart_handle(uart), data.data(), length);
        return false;  // Placeholder for actual implementation
    }

    /**
     * @brief Abort receiving data on the UART peripheral.
     * @param[in] uart The UART peripheral to abort receiving data on.
     */
    void UART::abort_receive(Peripheral uart) {
        HAL_UART_AbortReceive(get_hal_uart_handle(uart));
    }

    /**
     * @brief Abort transmitting data on the UART peripheral.
     * @param[in] uart The UART peripheral to abort.
     */
    void UART::abort_transmit(Peripheral uart) {
        HAL_UART_AbortTransmit(get_hal_uart_handle(uart));
    }
}  // namespace MW_UART

namespace MW_RTOS {
    /**
     * @brief Delay until a certain tick count.
     * @param[in,out] previous_wake Pointer to the previous wake tick.
     * @param[in] ms Milliseconds to delay.
     */
    void RTOS::delay_until(uint32_t* previous_wake, uint32_t ms) {
        *previous_wake = xTaskGetTickCount();
        TickType_t delay_ticks = pdMS_TO_TICKS(ms);
        vTaskDelayUntil(previous_wake, delay_ticks);
    }

    /**
     * @brief Delay for a certain number of milliseconds.
     * @param[in] ms Milliseconds to delay.
     */
    void RTOS::delay(uint32_t ms) {
        TickType_t delay_ticks = pdMS_TO_TICKS(ms);
        vTaskDelay(delay_ticks);
    }

    /**
     * @brief Get the current RTOS tick.
     * @return The current tick count.
     */
    TickType RTOS::get_current_tick() {
        return static_cast<TickType>(HAL_GetTick());
    }

    /**
     * @brief Create a queue.
     * @param[out] queue The created queue handle.
     * @param[in] queue_length Number of items in the queue.
     * @param[in] item_size Size of each item.
     * @return true if creation failed, false otherwise.
     */
    bool RTOS::queue_create(QueueHandle& queue, size_t queue_length,
                            size_t item_size) {
        queue = xQueueCreate(queue_length, item_size);
        return queue == nullptr;
    }

    /**
     * @brief Overwrite the queue with new data.
     * @param[in] queue The queue handle.
     * @param[in] data_ptr Pointer to the data to write.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_overwrite(QueueHandle queue, void* data_ptr) {
        return xQueueOverwrite(queue, data_ptr) == pdTRUE;
    }

    /**
     * @brief Push data to the back of the queue.
     * @param[in] queue The queue handle.
     * @param[in] data_ptr Pointer to the data to write.
     * @param[in] ticks_to_wait How long to wait if the queue is full.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_pushback(QueueHandle queue, void* data_ptr,
                              TickType ticks_to_wait) {
        return xQueueSendToBack(queue, data_ptr, ticks_to_wait) == pdTRUE;
    }

    /**
     * @brief Overwrite the queue from ISR.
     * @param[in] queue The queue handle.
     * @param[in] data_ptr Pointer to the data to write.
     * @param[out] awakenHigherPrio Set to true if a higher priority task should be woken.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_overwrite_from_isr(QueueHandle queue, void* data_ptr,
                                        bool* awakenHigherPrio) {
        BaseType_t awaken;
        BaseType_t result = xQueueOverwriteFromISR(queue, data_ptr, &awaken);
        if (awakenHigherPrio != nullptr) {
            *awakenHigherPrio = awaken == pdTRUE;
        }
        return result == pdTRUE;
    }

    /**
     * @brief Push data to the back of the queue from ISR.
     * @param[in] queue The queue handle.
     * @param[in] data_ptr Pointer to the data to write.
     * @param[out] awakenHigherPrio Set to true if a higher priority task should be woken.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_pushback_from_isr(QueueHandle queue, void* data_ptr,
                                       bool* awakenHigherPrio) {
        BaseType_t awaken;
        BaseType_t result = xQueueSendFromISR(queue, data_ptr, &awaken);
        if (awakenHigherPrio != nullptr) {
            *awakenHigherPrio = awaken == pdTRUE;
        }
        return result == pdTRUE;
    }

    /**
     * @brief Peek at the front of the queue.
     * @param[in] queue The queue handle.
     * @param[out] data_ptr Pointer to the data to read.
     * @param[in] ticks_to_wait How long to wait if the queue is empty.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_peek(QueueHandle queue, void* data_ptr,
                          TickType ticks_to_wait) {
        return xQueuePeek(queue, data_ptr, ticks_to_wait) == pdTRUE;
    }

    /**
     * @brief Get data from the front of the queue.
     * @param[in] queue The queue handle.
     * @param[out] data_ptr Pointer to the data to read.
     * @param[in] ticks_to_wait How long to wait if the queue is empty.
     * @return true if successful, false otherwise.
     */
    bool RTOS::queue_get(QueueHandle queue, void* data_ptr,
                         TickType ticks_to_wait) {
        return xQueueReceive(queue, data_ptr, ticks_to_wait) == pdTRUE;
    }
}  // namespace MW_RTOS
