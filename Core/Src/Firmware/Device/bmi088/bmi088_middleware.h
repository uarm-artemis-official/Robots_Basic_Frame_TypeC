/*******************************************************************************
* @file           : bmi088_middleware.h
* @brief          : gyroscope BMI088 read/write for attitude reading
* @restructed     : Nov, 2023
* @maintainer     : Haoran
********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __BMI088_MIDDLEWARE_H__
#define __BMI088_MIDDLEWARE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32f407xx.h"
#include "tim.h"

extern void BMI088_GPIO_Init(void);
extern void BMI088_Com_Init(void);
extern void BMI088_Delay_ms(uint16_t ms);
extern void BMI088_Delay_us(uint16_t us);

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_Read_Write_Byte(uint8_t reg);
uint8_t BMI088_Gyro_RW_Byte(uint8_t* rx, uint8_t* tx, uint8_t size);
void BMI088_Set_PWM_Duty_Cycle(uint16_t on_duration);

#ifdef __cplusplus
}
#endif

#endif
