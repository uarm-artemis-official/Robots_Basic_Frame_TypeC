#ifndef __INIT_ISRS_HPP
#define __INIT_ISRS_HPP

#include <memory>
#include "can.h"
#include "can_isr.hpp"
#include "middleware_interfaces.hpp"
#include "tim.h"
#include "uart_isr.hpp"
#include "usart.h"

static std::shared_ptr<isr::can::CAN_ISR> can_isr;
static std::shared_ptr<isr::uart::UART_ISR> uart_isr;

bool isr::can::install_isr(CAN_ISR& can_isr_ref) {
    can_isr = std::make_shared<isr::can::CAN_ISR>(can_isr_ref);
    return true;
}

bool isr::uart::install_isr(UART_ISR& uart_isr_ref) {
    uart_isr = std::make_shared<isr::uart::UART_ISR>(uart_isr_ref);
    return true;
}

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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (!can_isr || !can_isr->is_initialized()) {
        return;
    }

    MW_CAN::BUS bus;

    // TODO: Refactor in cause of using FIFO1 in the future.
    uint32_t frame_ide =
        CAN_RI0R_IDE & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR;
    bool is_extended_id = frame_ide == CAN_ID_EXT;
    if (hcan == &hcan1) {
        bus = is_extended_id ? MW_CAN::BUS::CAN_1B : MW_CAN::BUS::CAN_1;
    } else if (hcan == &hcan2) {
        bus = is_extended_id ? MW_CAN::BUS::CAN_2B : MW_CAN::BUS::CAN_2;
    } else {
        ASSERT(false, "Received message on unknown hcan.");
    }
    can_isr->run_isr_routines(isr::can::ECallbacks::MESSAGE_PENDING, bus);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (!uart_isr || !uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral;
    if (huart == &huart1) {
        peripheral = MW_UART::Peripheral::UART1;
    } else if (huart == &huart3) {
        peripheral = MW_UART::Peripheral::UART3;
    } else {
        ASSERT(false, "Receive complete on unknown huart.");
    }

    uart_isr->run_isr_routines(isr::uart::ECallbacks::RECEIVE_COMPLETE,
                               peripheral);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (!uart_isr || !uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral;
    if (huart == &huart1) {
        peripheral = MW_UART::Peripheral::UART1;
    } else if (huart == &huart3) {
        peripheral = MW_UART::Peripheral::UART3;
    } else {
        ASSERT(false, "Receive complete on unknown huart.");
    }

    uart_isr->run_isr_routines(isr::uart::ECallbacks::ON_ERROR, peripheral);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    if (!uart_isr || !uart_isr->is_initialized()) {
        return;
    }

    MW_UART::Peripheral peripheral;
    if (huart == &huart1) {
        peripheral = MW_UART::Peripheral::UART1;
    } else if (huart == &huart3) {
        peripheral = MW_UART::Peripheral::UART3;
    } else {
        ASSERT(false, "Transmit complete on unknown huart.");
    }

    uart_isr->run_isr_routines(isr::uart::ECallbacks::TRANSMIT_COMPLETE,
                               peripheral);
}

#endif