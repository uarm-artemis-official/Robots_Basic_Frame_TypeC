#ifndef __COMMS_TYPES_H
#define __COMMS_TYPES_H

#ifdef GTEST
  #include "stdint.h"

  // Ripped from stm32f4xx_hal_can.h
  typedef struct
  {
    uint32_t StdId;    /*!< Specifies the standard identifier.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint32_t ExtId;    /*!< Specifies the extended identifier.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF. */

    uint32_t IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                            This parameter can be a value of @ref CAN_identifier_type */

    uint32_t RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                            This parameter can be a value of @ref CAN_remote_transmission_request */

    uint32_t DLC;      /*!< Specifies the length of the frame that will be transmitted.
                            This parameter must be a number between Min_Data = 0 and Max_Data = 8. */

    FunctionalState TransmitGlobalTime; /*!< Specifies whether the timestamp counter value captured on start
                            of frame transmission, is sent in DATA6 and DATA7 replacing pData[6] and pData[7].
                            @note: Time Triggered Communication Mode must be enabled.
                            @note: DLC must be programmed as 8 bytes, in order these 2 bytes are sent.
                            This parameter can be set to ENABLE or DISABLE. */

  } CAN_TxHeaderTypeDef;
#else
  #include "stm32f407xx.h"
  #include "stm32f4xx_hal.h"
#endif

/* =========================================================================
 * QUEUE M TYPES
 * ====================================================================== */
/**
  * @brief  queue management main struct
  */
typedef struct{
	int16_t head; //queue head counter/pointer
	int16_t tail;
}QueueManage_t;

/* can fifo queue */
typedef struct
{
    CAN_TxHeaderTypeDef header;
    uint8_t data[8];
} CanMessage_t;

#endif