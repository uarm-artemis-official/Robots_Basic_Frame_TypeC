#include "uart_isr.hpp"
#include "cmsis_os.h"
#include "message_center.hpp"
#include "pack_handler.h"
#include "referee_data.h"
#include "usart.h"

static MessageCenter& message_center = MessageCenter::get_instance();
static UART_Config_t system_config = UART_NONE;

static uint8_t rc_frame_buffer[DBUS_BUFFER_LEN] = {0};
static uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE];
static uint8_t ref_rx_frame[MAX_REF_BUFFER_SIZE];

static uint32_t uart_complete_count = 0;
static uint32_t uart_error_count = 0;

// Add semaphores for synchronization
static SemaphoreHandle_t ref_rx_semaphore = NULL;
// static SemaphoreHandle_t pack_rx_semaphore = NULL;
// static SemaphoreHandle_t rc_rx_semaphore = NULL;

void init_uart_isr(UART_Config_t config) {
    system_config = config;

    // Create semaphores
    ref_rx_semaphore = xSemaphoreCreateBinary();
    // pack_rx_semaphore = xSemaphoreCreateBinary();
    // rc_rx_semaphore = xSemaphoreCreateBinary();

    // Give initial semaphores
    xSemaphoreGive(ref_rx_semaphore);
    // xSemaphoreGive(pack_rx_semaphore);
    // xSemaphoreGive(rc_rx_semaphore);

    switch (config) {
        case CHASSIS: {
            // Initialize UART with error handling
            if (HAL_UART_Receive_DMA(&huart3, rc_frame_buffer,
                                     DBUS_BUFFER_LEN) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
            if (HAL_UART_Receive_DMA(&huart1, ref_rx_frame,
                                     sizeof(ref_rx_frame)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
            break;
        }
        case GIMBAL: {
            uc_start_receive(pack_buffer, MAX_PACK_BUFFER_SIZE);
            break;
        }
        default: {
            break;
        }
    }
}

/*
 * @brief  UART DMA Callback function, update all the dma transmission IT here
 * @note   This function is called when：
 * 			 Referee system recv: UART3_DMA1_Stream1
 * 			 Mini PC recv: 		  UART6_DMA2_Stream1
 *
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart1 && system_config == CHASSIS) {
        // Check if we can take the semaphore
        if (xSemaphoreTakeFromISR(ref_rx_semaphore,
                                  &xHigherPriorityTaskWoken) == pdTRUE) {
            // Publish message and restart DMA
            message_center.pub_message_from_isr(REFEREE_IN, ref_rx_frame, NULL);
            // Clear buffer before restarting DMA
            memset(ref_rx_frame, 0, sizeof(ref_rx_frame));
            if (HAL_UART_Receive_DMA(&huart1, ref_rx_frame,
                                     sizeof(ref_rx_frame)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }

            // Give back the semaphore
            xSemaphoreGiveFromISR(ref_rx_semaphore, &xHigherPriorityTaskWoken);
        }
    } else if (huart == &huart1 && system_config == GIMBAL) {
        message_center.pub_message_from_isr(UC_PACK_IN, pack_buffer, NULL);
        HAL_UART_Receive_DMA(&huart1, pack_buffer, MAX_PACK_BUFFER_SIZE);
    } else if (huart == &huart3 && system_config == CHASSIS) {
        uart_complete_count = (uart_complete_count + 1) % 1000000;
        message_center.pub_message_from_isr(RC_RAW, rc_frame_buffer, NULL);
        HAL_UART_Receive_DMA(&huart3, rc_frame_buffer, DBUS_BUFFER_LEN);
    }

    // If a higher priority task was woken, yield
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart3 && system_config == CHASSIS) {
        uart_error_count = (uart_error_count + 1) % 100000;
        HAL_UART_Receive_DMA(&huart3, rc_frame_buffer, DBUS_BUFFER_LEN);
        // TODO: Implement error handling.
    } else if (huart == &huart1) {
        // Handle huart1 errors
        HAL_UART_DMAStop(huart);

        // Clear error flags
        __HAL_UART_CLEAR_FLAG(
            huart, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE);

        if (system_config == CHASSIS) {
            // Clear buffer
            memset(ref_rx_frame, 0, sizeof(ref_rx_frame));

            // Restart DMA
            if (HAL_UART_Receive_DMA(&huart1, ref_rx_frame,
                                     sizeof(ref_rx_frame)) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        } else if (system_config == GIMBAL) {
            // Clear buffer
            memset(pack_buffer, 0, MAX_PACK_BUFFER_SIZE);

            // Restart DMA
            if (HAL_UART_Receive_DMA(&huart1, pack_buffer,
                                     MAX_PACK_BUFFER_SIZE) != HAL_OK) {
                // Handle error
                Error_Handler();
            }
        }
    }
}