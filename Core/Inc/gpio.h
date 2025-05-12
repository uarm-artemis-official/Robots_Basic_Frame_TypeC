/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "public_defines.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
// TODO: Consider refactoring GPIO and other peripherals to adhere to dependency inversion.
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
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

