#include "stddef.h"
#include "stdint.h"
#include "string.h"

#include "can_isr.hpp"
#include "dji_motor.h"
#include "lk_motor.h"

// TODO: Make ISRs into "Application" like classes and declare their
// usage in Callbacks in main.cpp.

namespace CAN_ISR {
    void CAN_ISR::init(Config _config) {
        config = _config;
    }

    uint8_t CAN_ISR::get_free_buffer(uint32_t stdId) {
        uint8_t free_index = 0xff;
        for (int i = 0; i < 8; i++) {
            if (motor_read.can_ids[i] == 0)
                free_index = i;
            if (motor_read.can_ids[i] == stdId) {
                return i;
            }
        }
        return free_index;
    }

    void CAN_ISR::read_motor_data(CAN_HandleTypeDef* hcan,
                                  CAN_RxHeaderTypeDef& rx_header) {
        uint8_t buffer_index = get_free_buffer(rx_header.StdId);
        if (buffer_index < MAX_MOTOR_COUNT) {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                                 motor_read.feedback[buffer_index]);
            motor_read.can_ids[buffer_index] =
                static_cast<Motor_CAN_ID_t>(rx_header.StdId);
            mc.pub_message_from_isr(motor_read);
        }
    }

    void CAN_ISR::on_message_pending(CAN_HandleTypeDef* hcan) {
        CAN_RxHeaderTypeDef rx_header;
        rx_header.StdId =
            (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >>
            CAN_TI0R_STID_Pos;
        if (hcan == &hcan1) {
            read_motor_data(hcan, rx_header);
        }

        if (hcan == &hcan2) {
            mc2::CommIn comm_in;
            comm_in.topic_name = rx_header.StdId;

            if (config == Config::SENTRY_CHASSIS &&
                SWERVE_STEER_MOTOR1 <= rx_header.StdId &&
                rx_header.StdId <= SWERVE_STEER_MOTOR4) {
                read_motor_data(hcan, rx_header);
            } else {
                // TODO: Add removal/filters for non-supported CAN2 messages
                HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header,
                                     comm_in.bytes.data());
                // TODO: Store messages in RTOS queue and implement API for accessing messages.
                mc.pub_message_from_isr(comm_in);
            }
        }
    }
}  // namespace CAN_ISR
