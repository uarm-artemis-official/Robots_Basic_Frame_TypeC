#include "uart_isr.hpp"
#include <cstring>
#include "cmsis_os.h"
#include "message_center.hpp"
#include "pack_handler.h"
#include "subsystems_classes.hpp"
#include "subsystems_defines.hpp"
#include "usart.h"

// TODO: Remove direct usage of constants like DBUS_BUFFER_LEN in code.
namespace UART_ISR {
    UART_ISR::UART_ISR(mc2::RobotMC& mc_ref) : mc(mc_ref) {}

    void UART_ISR::init(Config _config) {
        config = _config;

        std::memset(state.rc_raw.rc_bytes.data(), 0,
                    sizeof(state.rc_raw.rc_bytes));
        std::memset(state.uc_pack_in.bytes.data(), 0,
                    sizeof(state.uc_pack_in.bytes));
        std::memset(state.referee_in.ref_bytes.data(), 0,
                    sizeof(state.referee_in.ref_bytes));

        if (config == Config::CHASSIS) {
            if (HAL_UART_Receive_DMA(&huart1, state.referee_in.ref_bytes.data(),
                                     sizeof(state.referee_in.ref_bytes)) !=
                HAL_OK) {
                // Handle error
                Error_Handler();
            }
        }

        if (config == Config::CHASSIS || config == Config::AUTO_AIM) {
            // Initialize UART with error handling
            if (HAL_UART_Receive_DMA(&huart3, state.rc_raw.rc_bytes.data(),
                                     sizeof(state.rc_raw.rc_bytes)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        }

        if (config == Config::GIMBAL || config == Config::AUTO_AIM) {
            uc_start_receive(state.uc_pack_in.bytes.data(),
                             sizeof(state.uc_pack_in.bytes));
        }
    }

    void UART_ISR::on_receive_complete(UART_HandleTypeDef* huart) {
        if (huart == &huart1 && config == Config::CHASSIS) {
            // Publish message and restart DMA
            mc.pub_message_from_isr(state.referee_in);
            // Clear buffer before restarting DMA
            memset(state.referee_in.ref_bytes.data(), 0,
                   sizeof(state.referee_in.ref_bytes));
            if (HAL_UART_Receive_DMA(&huart1, state.referee_in.ref_bytes.data(),
                                     sizeof(state.referee_in.ref_bytes)) !=
                HAL_OK) {
                // Handle error
                Error_Handler();
            }
        } else if (huart == &huart1 &&
                   (config == Config::GIMBAL || config == Config::AUTO_AIM)) {
            mc.pub_message_from_isr(state.uc_pack_in);
            HAL_UART_Receive_DMA(&huart1, state.uc_pack_in.bytes.data(),
                                 MAX_PACK_BUFFER_SIZE);
        } else if (huart == &huart3 &&
                   (config == Config::CHASSIS || config == Config::AUTO_AIM)) {
            state.complete_count = (state.complete_count + 1) % 1000000;
            mc.pub_message_from_isr(state.rc_raw);
            HAL_UART_Receive_DMA(&huart3, state.rc_raw.rc_bytes.data(),
                                 DBUS_BUFFER_LEN);
        }
    }

    void UART_ISR::on_error(UART_HandleTypeDef* huart) {
        if (huart == &huart3 && config == Config::CHASSIS) {
            state.error_count = (state.error_count + 1) % 100000;
            HAL_UART_Receive_DMA(&huart3, state.rc_raw.rc_bytes.data(),
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
                memset(state.referee_in.ref_bytes.data(), 0,
                       sizeof(state.referee_in.ref_bytes));

                // Restart DMA
                if (HAL_UART_Receive_DMA(
                        &huart1, state.referee_in.ref_bytes.data(),
                        sizeof(state.referee_in.ref_bytes)) != HAL_OK) {
                    // Handle error
                    Error_Handler();
                }
            } else if (config == Config::GIMBAL) {
                // Clear buffer
                memset(state.uc_pack_in.bytes.data(), 0, MAX_PACK_BUFFER_SIZE);

                // Restart DMA
                if (HAL_UART_Receive_DMA(&huart1, state.uc_pack_in.bytes.data(),
                                         MAX_PACK_BUFFER_SIZE) != HAL_OK) {
                    // Handle error
                    Error_Handler();
                }
            }
        }
    }
}  // namespace UART_ISR