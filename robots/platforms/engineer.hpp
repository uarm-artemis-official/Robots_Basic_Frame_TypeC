#include "lk_motor_driver.hpp"
#include "middleware_classes.hpp"
#include "usart.h"

static MW_GPIO::GPIO gpio;
static MW_BASE::Base base;
static MW_UART::UART uart;
static MW_CAN::CAN can;

bool init_firmware() {
    bool gpio_status = gpio.init();
    bool base_status = base.init();
    bool can_status = can.init();
    return gpio_status && base_status && can_status;
}

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void can_filter_enable(CAN_HandleTypeDef* hcan) {
    CAN_FilterTypeDef CAN_FilterConfigStructure;

    CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_16BIT;
    CAN_FilterConfigStructure.FilterActivation = ENABLE;
    if (hcan == &hcan1) {
        CAN_FilterConfigStructure.FilterBank = 0;
    } else if (hcan == &hcan2) {
        CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
        CAN_FilterConfigStructure.FilterBank = 14;
    }

    bool filter_status =
        HAL_CAN_ConfigFilter(hcan, &CAN_FilterConfigStructure) == HAL_OK;
    // activate the canx msg callback interrupt
    bool notification_status = HAL_CAN_ActivateNotification(
                                   hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK;
    ASSERT(filter_status && notification_status,
           "Failed to configure CAN filter and activate notification.");
}

static bool responded = true;
static uint32_t angle = 0;
void main_cpp() {
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_1,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_3,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_5,
                   MW_GPIO::State::HIGH);
    gpio.write_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_7,
                   MW_GPIO::State::HIGH);

    CAN_FilterTypeDef can1_filter_config;
    can1_filter_config.FilterIdHigh = 0x0000;
    can1_filter_config.FilterIdLow = 0x0000;
    can1_filter_config.FilterMaskIdHigh = 0x0000;
    can1_filter_config.FilterMaskIdLow = 0x0000;
    can1_filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can1_filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    can1_filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
    can1_filter_config.FilterActivation = ENABLE;
    can1_filter_config.FilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan1, &can1_filter_config);

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan1);

    while (true) {
        MW_CAN::CANFrame can_tx_message;
        // lk_motor::format_off_message(0x141, can_tx_message);
        // can.send_data(
        //     MW_CAN::BUS::CAN_1, can_tx_message.sid, can_tx_message.eid,
        //     reinterpret_cast<const uint8_t*>(can_tx_message.payload.data()),
        //     can_tx_message.dlc);

        uint16_t speed_limit = 1500;

        if (responded) {
            uint8_t payload[8];
            payload[0] = 0xA6;
            payload[1] = 0;
            payload[2] = speed_limit & 0xFF;
            payload[3] = (speed_limit >> 8) & 0xFF;
            payload[4] = angle & 0xFF;
            payload[5] = (angle >> 8) & 0xFF;
            payload[6] = (angle >> 16) & 0xFF;
            payload[7] = (angle >> 24) & 0xFF;

            CAN_TxHeaderTypeDef tx_header;
            tx_header.StdId = 0x141;
            tx_header.ExtId = 0;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8;
            uint32_t mailbox;
            HAL_StatusTypeDef status =
                HAL_CAN_AddTxMessage(&hcan1, &tx_header, payload, &mailbox);
            ASSERT(status == HAL_OK, "Failed to send CAN message.");

            responded = false;
            angle = (angle + 1000) % 36000;
            HAL_Delay(50);
        } else {
            uint8_t payload[8];
            payload[0] = 0x81;
            payload[1] = 0;
            payload[2] = 0;
            payload[3] = 0;
            payload[4] = 0;
            payload[5] = 0;
            payload[6] = 0;
            payload[7] = 0;

            CAN_TxHeaderTypeDef tx_header;
            tx_header.StdId = 0x141;
            tx_header.ExtId = 0;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 8;
            uint32_t mailbox;
            HAL_StatusTypeDef status =
                HAL_CAN_AddTxMessage(&hcan1, &tx_header, payload, &mailbox);
            ASSERT(status == HAL_OK, "Failed to send CAN message.");
            // angle = (angle + 1000) % 36000;
            HAL_Delay(50);
        }
    }
}

extern "C" {
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
}

static size_t rx_count = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    MW_CAN::CANFrame can_rx_message;
    if (hcan->Instance == CAN1) {
        uint32_t dlc;
        can.receive_data(
            MW_CAN::BUS::CAN_1, MW_CAN::FIFO::FIFO_0, can_rx_message.sid,
            can_rx_message.eid,
            reinterpret_cast<uint8_t*>(can_rx_message.payload.data()), dlc);
        gpio.toggle_pin(MW_GPIO::Port::PORT_G, MW_GPIO::Pin::PIN_1);
        rx_count++;
        responded = true;
    }
}
