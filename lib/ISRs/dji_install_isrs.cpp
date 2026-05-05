#ifndef __INIT_ISRS_HPP
#define __INIT_ISRS_HPP

#include <memory>
#include "can_isr.hpp"
#include "mappings/mapping_api.hpp"
#include "middleware_interfaces.hpp"
#include "uart_isr.hpp"

#if defined(MW_ENABLE_TIM)
#include "tim.h"
#endif

#if defined(MW_ENABLE_CAN)
#include "can.h"
#endif

#if defined(MW_ENABLE_UART)
#include "usart.h"
#endif

static isr::can::CAN_ISR *installed_can_isr;
static isr::uart::UART_ISR* installed_uart_isr;

bool isr::can::install_isr(CAN_ISR* installed_can_isr_ref) {
    ASSERT(installed_can_isr_ref != nullptr, "Cannot install null CAN ISR.");
    installed_can_isr = installed_can_isr_ref;
    return true;
}

#if defined(MW_ENABLE_TIM)

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM5) {
        HAL_IncTick();
    }
}
#endif

#if defined(MW_ENABLE_CAN)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    // if (!installed_can_isr || !installed_can_isr->is_initialized()) {
    //     CAN_RxHeaderTypeDef dummy_frame;
    //     uint8_t data[8] = {0};
    //     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &dummy_frame, data);
    //     return;
    // }

    MW_CAN::BUS bus;

    // TODO: Refactor in cause of using FIFO1 in the future.
    uint32_t frame_ide =
        CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
    bool is_extended_id = frame_ide == CAN_ID_EXT;

    bus = platform::get_can_from_hal(hcan);
    ASSERT(bus != MW_CAN::BUS::Unknown, "Received message on unknown hcan.");

    if (is_extended_id) {
        if (bus == MW_CAN::BUS::CAN_1) {
            bus = MW_CAN::BUS::CAN_1B;
        } else if (bus == MW_CAN::BUS::CAN_2) {
            bus = MW_CAN::BUS::CAN_2B;
        } else {
            ASSERT(false, "Invalid bus for extended ID message.");
        }
    }

    installed_can_isr->run_isr_routines(isr::can::ECallbacks::MESSAGE_PENDING, bus);
}
#endif

#if defined(MW_ENABLE_UART)

bool isr::uart::install_isr(UART_ISR* installed_uart_isr_ref) {
    ASSERT(installed_uart_isr_ref != nullptr, "Cannot install null UART ISR.");
    installed_uart_isr = installed_uart_isr_ref;
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (!installed_uart_isr || !installed_uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral = platform::get_uart_from_hal(huart);
    ASSERT(peripheral != MW_UART::Peripheral::Unknown,
           "Receive complete on unknown huart.");

    installed_uart_isr->run_isr_routines(isr::uart::ECallbacks::RECEIVE_COMPLETE,
                               peripheral);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (!installed_uart_isr || !installed_uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral = platform::get_uart_from_hal(huart);
    ASSERT(peripheral != MW_UART::Peripheral::Unknown,
           "Error on unknown huart.");

    installed_uart_isr->run_isr_routines(isr::uart::ECallbacks::ON_ERROR, peripheral);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (!installed_uart_isr || !installed_uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral = platform::get_uart_from_hal(huart);
    ASSERT(peripheral != MW_UART::Peripheral::Unknown,
           "Transmit complete on unknown huart.");

    installed_uart_isr->run_isr_routines(isr::uart::ECallbacks::TRANSMIT_COMPLETE,
                               peripheral);
}

#endif

#endif
