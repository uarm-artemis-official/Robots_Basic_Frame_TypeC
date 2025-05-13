#include "uart_isr.h"
#include "message_center.h"
#include "pack_handler.h"
#include "usart.h"

static MessageCenter& message_center = MessageCenter::get_instance();
static UART_Config_t system_config = UART_NONE;

static uint8_t rc_frame_buffer[DBUS_BUFFER_LEN] = {0};
static uint8_t pack_buffer[MAX_PACK_BUFFER_SIZE];

static uint32_t uart_complete_count = 0;
static uint32_t uart_error_count = 0;

void init_uart_isr(UART_Config_t config) {
    system_config = config;

    switch (config) {
        case CHASSIS: {
            HAL_UART_Receive_DMA(&huart3, rc_frame_buffer, DBUS_BUFFER_LEN);
            // TODO: Uncomment when referee system is supported.
            // HAL_UART_Receive_DMA(&huart1, ref_rx_frame, sizeof(ref_rx_frame));
            break;
        }
        case GIMBAL: {
            start_receive(pack_buffer);
            break;
        }
        default: {
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
    if (huart == &huart1 && system_config == CHASSIS) {
        /* re-activate DMA */
        //		referee_parsed_flag = 1;
    } else if (huart == &huart1 && system_config == GIMBAL) {
        message_center.pub_message_from_isr(UC_PACK_IN, pack_buffer, NULL);
        HAL_UART_Receive_DMA(&huart1, pack_buffer, MAX_PACK_BUFFER_SIZE);
    } else if (huart == &huart3 && system_config == CHASSIS) {
        uart_complete_count = (uart_complete_count + 1) % 1000000;
        message_center.pub_message_from_isr(RC_RAW, rc_frame_buffer, NULL);
        HAL_UART_Receive_DMA(&huart3, rc_frame_buffer, DBUS_BUFFER_LEN);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart3 && system_config == CHASSIS) {
        uart_error_count = (uart_error_count + 1) % 100000;
        HAL_UART_Receive_DMA(&huart3, rc_frame_buffer, DBUS_BUFFER_LEN);
        // TODO: Implement error handling.
    }
}