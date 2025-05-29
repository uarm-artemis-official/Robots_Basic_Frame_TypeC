/*******************************************************************************
* @file           : bmi088_driver.h
* @brief          : gyroscope BMI088 and ist8310 read/write for attitude reading
* @restructed     : Nov, 2023
* @maintainer     : Haoran
********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __BMI088_DRIVER_H__
#define __BMI088_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi088_middleware.h"
#include "bmi088_reg.h"
#include "stdint.h"

/* public defines of IMU */
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.000091552734375f
#define BMI088_ACCEL_6G_SEN 0.00018310546875f
#define BMI088_ACCEL_12G_SEN 0.0003662109375f
#define BMI088_ACCEL_24G_SEN 0.000732421875f

#define BMI088_GYRO_2000_SEN 0.06103515625f
#define BMI088_GYRO_1000_SEN 0.030517578125f
#define BMI088_GYRO_500_SEN 0.0152587890625f
#define BMI088_GYRO_250_SEN 0.00762939453125f
#define BMI088_GYRO_125_SEN 0.00381469726563f

#define DEGREES_TO_RADIANS 0.0174532925199f

// TODO: Change when moving locations
// Edmonton gravity conversion
#define G_TO_METERS_PER_SECOND_SQUARED 9.8115861

typedef struct BMI088_REAL_DATA {
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;

enum {
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
};

/* define private functions of IMU */
uint8_t BMI088_init(void);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);
uint8_t bmi088_accel_self_test(void);
uint8_t bmi088_gyro_self_test(void);
void BMI088_read_gyro_who_am_i(void);
uint8_t BMI088_read_accel_who_am_i(void);
void BMI088_accel_read(uint8_t* rx_buf, float accel[3], float* time);
void BMI088_gyro_read(uint8_t* rx_buf, float gyro[3]);
void BMI088_temperature_read(uint8_t* rx_buf, float* temperate);
void BMI088_Read(float gyro[3], float accel[3], float* temperate);
float get_BMI088_temperature(void);
void get_BMI088_gyro(int16_t gyro[3]);
void get_BMI088_accel(float accel[3]);

#define BMI088_ACCEL_Read_Single_Reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80);   \
        BMI088_Read_Write_Byte(0x55);           \
        (data) = BMI088_Read_Write_Byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

#ifdef __cplusplus
}
#endif

#endif
