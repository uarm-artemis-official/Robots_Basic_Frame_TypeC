#ifndef __MIDDLEWARE_TYPES_HPP
#define __MIDDLEWARE_TYPES_HPP

#include "uarm_types.hpp"

namespace MW_GPIO {
    enum class Pin {
        PIN_0,
        PIN_1,
        PIN_2,
        PIN_3,
        PIN_4,
        PIN_5,
        PIN_6,
        PIN_7,
        PIN_8,
        PIN_9,
        PIN_10,
        PIN_11,
        PIN_12,
        PIN_13,
        PIN_14,
        PIN_15,
    };

    enum class Port {
        PORT_A,
        PORT_B,
        PORT_C,
        PORT_D,
        PORT_E,
        PORT_F,
        PORT_G,
        PORT_H,
        PORT_I,
        PORT_J,
        PORT_K,
    };

    enum class State { LOW, HIGH };
}  // namespace MW_GPIO

namespace MW_TIM {
    enum class Timer {
        TIM_1,
        TIM_4,
        TIM_5,
        TIM_8,
        TIM_10,
        TIM_13,
    };

    enum class Channel {
        CHANNEL_1,
        CHANNEL_2,
        CHANNEL_3,
        CHANNEL_4,
    };
}  // namespace MW_TIM

namespace MW_CAN {
    enum class BUS {
        CAN_1,
        CAN_2,
    };

    enum class Notification {
        RX_FIFO0_MSG_PENDING,
    };

    enum class FilterMode {
        IDMask,
        IDList,
    };

    enum class FIFO {
        FIFO_0,
        FIFO_1,
    };

    /**
     * @brief Filter configuration for CAN messages.
     * This structure defines the filter settings for CAN messages, including ID, mask, FIFO assignment
     * and activation status.
     */
    struct Filter {
        uint32_t id_high;      /**< The high part of the CAN filter ID */
        uint32_t id_low;       /**< The low part of the CAN filter ID */
        uint32_t mask_id_high; /**< The high part of the CAN filter mask */
        uint32_t mask_id_low;  /**< The low part of the CAN filter mask */
        FIFO
            fifo_assignment; /**< FIFO assignment for the filter (FIFO_0 or FIFO_1) */
        FilterMode mode; /**< Filter mode (IDMask or IDList) */
        bool
            is_activated; /**< Activation status of the filter (true for ENABLE, false for DISABLE) */
        uint32_t filter_bank; /**< Filter bank number */
        uint32_t
            slave_start_filter_bank; /**< Start filter bank for slave CAN instance (used for dual CAN). */
    };

}  // namespace MW_CAN

namespace MW_UART {
    enum class Peripheral {
        UART1,
        UART3,
        UART6,
    };
}  // namespace MW_UART

#endif  // __MIDDLEWARE_TYPES_HPP