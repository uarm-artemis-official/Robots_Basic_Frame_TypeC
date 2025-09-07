#include "uart_isr.hpp"
#include <cstring>
#include "cmsis_os.h"
#include "pack_handler.h"
#include "referee_data.h"
#include "subsystems_classes.hpp"
#include "subsystems_defines.hpp"
#include "usart.h"

// Add semaphores for synchronization
namespace UART_ISR {
    UART_ISR::UART_ISR(IMessageCenter& _message_center)
        : message_center(_message_center) {}

    void UART_ISR::init(Config _config) {
        config = _config;

        std::memset(state.rc_frame_buffer, 0, sizeof(state.rc_frame_buffer));
        std::memset(state.pack_buffer, 0, sizeof(state.pack_buffer));
        std::memset(state.ref_rx_frame, 0, sizeof(state.ref_rx_frame));

        if (config == Config::CHASSIS) {
            if (HAL_UART_Receive_DMA(&huart1, state.ref_rx_frame,
                                     sizeof(state.ref_rx_frame)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        }

        if (config == Config::CHASSIS || config == Config::AUTO_AIM) {
            // Initialize UART with error handling
            if (HAL_UART_Receive_DMA(&huart3, state.rc_frame_buffer,
                                     DBUS_BUFFER_LEN) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        }

        if (config == Config::GIMBAL || config == Config::AUTO_AIM) {
            uc_start_receive(state.pack_buffer, MAX_PACK_BUFFER_SIZE);
        }
    }

    void UART_ISR::on_receive_complete(UART_HandleTypeDef* huart) {
        if (huart == &huart1 && config == Config::CHASSIS) {
            // Publish message and restart DMA
            message_center.pub_message_from_isr(REFEREE_IN, state.ref_rx_frame,
                                                NULL);
            // Clear buffer before restarting DMA
            memset(state.ref_rx_frame, 0, sizeof(state.ref_rx_frame));
            if (HAL_UART_Receive_DMA(&huart1, state.ref_rx_frame,
                                     sizeof(state.ref_rx_frame)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        } else if (huart == &huart1 &&
                   (config == Config::GIMBAL || config == Config::AUTO_AIM)) {
            message_center.pub_message_from_isr(UC_PACK_IN, state.pack_buffer,
                                                NULL);
            HAL_UART_Receive_DMA(&huart1, state.pack_buffer,
                                 MAX_PACK_BUFFER_SIZE);
        } else if (huart == &huart3 &&
                   (config == Config::CHASSIS || config == Config::AUTO_AIM)) {
            state.complete_count = (state.complete_count + 1) % 1000000;
            message_center.pub_message_from_isr(RC_RAW, state.rc_frame_buffer,
                                                NULL);
            HAL_UART_Receive_DMA(&huart3, state.rc_frame_buffer,
                                 DBUS_BUFFER_LEN);
        }
    }

    void UART_ISR::on_error(UART_HandleTypeDef* huart) {
        if (huart == &huart3 && config == Config::CHASSIS) {
            state.error_count = (state.error_count + 1) % 100000;
            HAL_UART_Receive_DMA(&huart3, state.rc_frame_buffer,
                                 DBUS_BUFFER_LEN);
            // TODO: Implement error handling.
        } else if (huart == &huart1) {
            // Handle huart1 errors
            HAL_UART_DMAStop(huart);

            // Clear error flags
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE |
                                             UART_FLAG_NE | UART_FLAG_PE);

            if (config == Config::CHASSIS) {
                // Clear buffer
                memset(state.ref_rx_frame, 0, sizeof(state.ref_rx_frame));

                // Restart DMA
                if (HAL_UART_Receive_DMA(&huart1, state.ref_rx_frame,
                                         sizeof(state.ref_rx_frame)) !=
                    HAL_OK) {
                    // Handle error
                    Error_Handler();
                }
            } else if (config == Config::GIMBAL) {
                // Clear buffer
                memset(state.pack_buffer, 0, MAX_PACK_BUFFER_SIZE);

                // Restart DMA
                if (HAL_UART_Receive_DMA(&huart1, state.pack_buffer,
                                         MAX_PACK_BUFFER_SIZE) != HAL_OK) {
                    // Handle error
                    Error_Handler();
                }
            }
        }
    }
}  // namespace UART_ISR