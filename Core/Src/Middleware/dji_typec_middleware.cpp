#include "can.h"
#include "middleware_interfaces.hpp"
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
     * @brief Implementation of IGPIO interface for DJI's Robomaster Type-C development boards.
     * GPIO pins are primarily used for toggling LEDs.
     */
    class GPIO : public IGPIO {
       public:
        /**
            * @brief Write a value to a GPIO pin.
            * @param port The GPIO port.
            * @param pin The GPIO pin.
            * @param state The desired state (HIGH or LOW) to write to the pin.
            */
        void write_pin(Port port, Pin pin, State state) override {
            HAL_GPIO_WritePin(get_hal_port(port), get_hal_pin(pin),
                              get_hal_state(state));
        }

        /**
            * @brief Read the value of a GPIO pin.
            * @param port The GPIO port.
            * @param pin The GPIO pin.
            * @return The current state (HIGH or LOW) of the pin.
            */
        State read_pin(Port port, Pin pin) override {

            return get_state_from_hal(
                HAL_GPIO_ReadPin(get_hal_port(port), get_hal_pin(pin)));
        }

        /**
            * @brief Toggle the state of a GPIO pin.
            * @param port The GPIO port.
            * @param pin The GPIO pin.
            */
        void toggle_pin(Port port, Pin pin) override {

            HAL_GPIO_TogglePin(get_hal_port(port), get_hal_pin(pin));
        }
    };
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
     * Implementation of IPWM for DJI's Robomaster Type C development board.
     */
    class PWM : public IPWM {
        /**
         * @brief Start PWM generation for a specific timer and channel.
         * @param[in] timer Timer to start.
         * @param[in] channel Channel of timer to start.
        */
        void start(Timer timer, Channel channel) {
            HAL_TIM_PWM_Start(get_hal_tim_handle(timer),
                              get_hal_tim_channel(channel));
        }

        /**
         * @brief Stop PWM generation for a specific timer and channel.
         * @param[in] timer Timer to stop.
         * @param[in] channel Channel of timer to stop.
         */
        void stop(Timer timer, Channel channel) {
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
        void set_duty_cycle(Timer timer, Channel channel, uint32_t compare) {
            __HAL_TIM_SET_COMPARE(get_hal_tim_handle(timer),
                                  get_hal_tim_channel(channel), compare);
        }
    };
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
     * @brief Implementation of ICAN interface for utilizing CAN buses on DJI's Type-C development board.
     * This class provides methods to send and receive CAN messages, start and stop the CAN bus,
     * activate notifications, and configure filters.
     */
    class CAN : public ICAN {
       public:
        /**
         * @brief Send data over the CAN bus.
         * @pre 0 < length <= 8.
         * @param[in] bus The CAN bus to send data on.
         * @param[in] id The ID of the message to send.
         * @param[in] data The data to send.
         * @param[in] length The length of the data.
         */
        void send_data(BUS bus, uint32_t id, const std::array<uint8_t, 8>& data,
                       uint32_t length) override {
            ASSERT(0 < length && length <= 8,
                   "0 bytes < CAN data length <= 8 bytes.");
            CAN_TxHeaderTypeDef tx_header;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = length;
            tx_header.StdId = id;

            HAL_CAN_AddTxMessage(get_hal_can_handle(bus), &tx_header,
                                 data.data(), nullptr);
        }

        /**
         * @brief Receive data from the CAN bus.
         * @param[in] bus The CAN bus to receive data from.
         * @param[in] fifo The FIFO to read from.
         * @param[out] id The ID of the received message.
         * @param[out] length The length of the received data.
         * @param[out] data The buffer to store the received data.
         */
        bool receive_data(BUS bus, FIFO fifo, uint32_t& id, uint32_t& length,
                          std::array<uint8_t, 8>& data) override {
            CAN_RxHeaderTypeDef rx_header;
            if (HAL_CAN_GetRxMessage(get_hal_can_handle(bus),
                                     get_hal_fifo(fifo), &rx_header,
                                     data.data()) != HAL_OK) {
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
        void start(BUS bus) override { HAL_CAN_Start(get_hal_can_handle(bus)); }

        /**
         * @brief Stop the CAN bus communication.
         * @param[in] bus The CAN bus to stop.
         */
        void stop(BUS bus) override { HAL_CAN_Stop(get_hal_can_handle(bus)); }

        /**
         * @brief Activate a CAN notification.
         * @param[in] bus The CAN bus to activate the notification on.
         * @param[in] notification The notification type to activate.
         */
        void activate_notification(BUS bus,
                                   Notification notification) override {
            switch (notification) {
                case Notification::RX_FIFO0_MSG_PENDING:
                    HAL_CAN_ActivateNotification(get_hal_can_handle(bus),
                                                 CAN_IT_RX_FIFO0_MSG_PENDING);
                    break;
                default:
                    ASSERT(false, "Unsupported CAN notification");
            }
        };

        /**
         * @brief Configure a CAN filter.
         * @param[in] bus The CAN bus to configure.
         * @param[in] filter_configuration The filter configuration to apply.
         */
        void configure_filter(BUS bus, Filter filter_configuration) override {
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

            HAL_CAN_ConfigFilter(get_hal_can_handle(bus), &filter_config);
        }
    };
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
     * @brief Implementation of IUART interface for utilizing UART on DJI's Type-C development board.
     * This class provides methods to send and receive UART messages, and abort reception and transmissions
     * of UART messages.
     */
    class UART : public IUART {
       public:
        /**
         * @brief Send data over UART.
         * @pre 0 < length <= MAX_UART_BUFFER_SIZE.
         * @param[in] uart The UART peripheral to send data on.
         * @param[in] data The data to send.
         * @param[in] length The length of the data.
         * @param[in] timeout The timeout for the transmission.
         */
        void send_data(Peripheral uart,
                       const std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                       uint32_t length, uint32_t timeout) override {
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
        bool receive_data(Peripheral uart,
                          std::array<uint8_t, MAX_UART_BUFFER_SIZE>& data,
                          uint32_t& length) override {
            ASSERT(0 < length && length <= MAX_UART_BUFFER_SIZE,
                   "UART data length must be > 0 and <= MAX_UART_BUFFER_SIZE");
            HAL_UART_Receive_DMA(get_hal_uart_handle(uart), data.data(),
                                 length);
            return false;  // Placeholder for actual implementation
        }

        /**
         * brief Abort receiving data on the UART peripheral.
         * @param[in] uart The UART peripheral to abort receiving data on.
         */
        void abort_receive(Peripheral uart) override {
            HAL_UART_AbortReceive(get_hal_uart_handle(uart));
        }

        /**
         * @brief Abort transmitting data on the UART peripheral.
         * @param[in] uart The UART peripheral to abort.
         */
        void abort_transmit(Peripheral uart) override {
            HAL_UART_AbortTransmit(get_hal_uart_handle(uart));
        }
    };
}  // namespace MW_UART