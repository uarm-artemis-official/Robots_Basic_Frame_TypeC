/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Servo_Motor_Pin_Pin GPIO_PIN_6
#define Servo_Motor_Pin_GPIO_Port GPIOI
#define Board_Status_Pin GPIO_PIN_1
#define Board_Status_GPIO_Port GPIOF
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define IMU_Heat_Pin_Pin GPIO_PIN_6
#define IMU_Heat_Pin_GPIO_Port GPIOF
#define LED_Red_Pin GPIO_PIN_12
#define LED_Red_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define LED_Green_Pin GPIO_PIN_11
#define LED_Green_GPIO_Port GPIOH
#define LED_Blue_Pin GPIO_PIN_10
#define LED_Blue_GPIO_Port GPIOH
#define Buzzer_Pin_Pin GPIO_PIN_14
#define Buzzer_Pin_GPIO_Port GPIOD
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define SOFTWARE_EXTI_Pin GPIO_PIN_0
#define SOFTWARE_EXTI_GPIO_Port GPIOG
#define SOFTWARE_EXTI_EXTI_IRQn EXTI0_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
#define HIGH_VOLT_Pin GPIO_PIN_15
#define HIGH_VOLT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
typedef _Bool bool_t;
#define MAX_REF_BUFFER_SZIE 256
#define MAX_REF_TX_DATA_LEN 128
#define VAL_LIMIT(val, min, max) \
  do                             \
  {                              \
    if ((val) <= (min))          \
    {                            \
      (val) = (min);             \
    }                            \
    else if ((val) >= (max))     \
    {                            \
      (val) = (max);             \
    }                            \
  } while (0)

#define ANGLE_LIMIT_360(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
  do                                \
  {                                 \
    if((val)>180)                   \
      (val) -= 360;                 \
  } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
  * @brief  Structure for record the board belongings
  * @note   indicate the current worked is either gimbal or chassis board
  * @param  None
  * @retval None
  */
typedef enum {
    GIMBAL_BOARD = 0,
    CHASSIS_BOARD = 1
} BoardStatusType;
BoardStatusType board_status;

typedef enum {
	PATROL_MODE = 0,
	DETECTION_MODE,
	AUTO_AIM_MODE,
	AUTO_PILOT_MODE,
	IDLE_MODE,
	DEBUG_MODE
}BoardMode_t; //only for sentry

typedef enum {
    GIMBAL_CENTER = 0, // relative angle control using encoder, chassis front always facing yaw center
    GIMBAL_FOLLOW,	   // relative angle control using encoder, chassis always moving along gimbal coordinate but not align center.
	SELF_GYRO, 		   // relative angle control using encoder, chassis keep spinning while gimbal can move freely
	INDPET_MODE,	   // chassis ground coordinate, or dummy version of self-gyro mode
}BoardActMode_t;	   // should be determined by remore controller

/* declare init function */
HAL_StatusTypeDef firmware_and_system_init(void);
extern char pdata[32]; //PACKLEN
extern int16_t referee_parsed_flag;
extern QueueHandle_t Ref_Pack_Queue;
//extern uint8_t rc_rx_buffer[18];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
