/*******************************************************************************
* @file           : bmi088driver.c
* @brief          : gyroscope BMI088 read/write for attitude reading
* @restructed     : Nov, 2023
* @maintainer     : Haoran
********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __BMI088_DRIVER_C__
#define __BMI088_DRIVER_C__

#include <bmi088_driver.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f
#define BMI088_Write_ACCEL_Reg_Num 6
#define BMI088_Write_GYRO_Reg_Num 6

static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data);
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t* return_data);
//static void BMI088_Write_Muli_Reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_Read_Muli_Reg(uint8_t reg, uint8_t* buf, uint8_t len);

#define BMI088_ACCEL_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_ACCEL_Read_Single_Reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80);   \
        BMI088_Read_Write_Byte(0x55);           \
        (data) = BMI088_Read_Write_Byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }

#define BMI088_ACCEL_Read_Muli_Reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_Read_Write_Byte((reg) | 0x80);      \
        BMI088_Read_Muli_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }

#define BMI088_GYRO_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_GYRO_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }

#define BMI088_GYRO_Read_Muli_Reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_Read_Muli_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static uint8_t
    write_BMI088_ACCEL_Reg_Data_ERROR[BMI088_Write_ACCEL_Reg_Num][3] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
         BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
         BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,
         BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
         BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL,
         BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP |
             BMI088_ACC_INT1_GPIO_LOW,
         BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,
         BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_GYRO_Reg_Data_ERROR[BMI088_Write_GYRO_Reg_Num][3] =
    {{BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
     {BMI088_GYRO_BANDWIDTH,
      BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
      BMI088_GYRO_BANDWIDTH_ERROR},
     {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
     {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
     {BMI088_GYRO_INT3_INT4_IO_CONF,
      BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
      BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
     {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3,
      BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t BMI088_init(void) {
    uint8_t ERROR = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_Init();
    BMI088_Com_Init();

    // self test pass and init
    if (bmi088_accel_self_test() != BMI088_NO_ERROR) {
        ERROR |= BMI088_SELF_TEST_ACCEL_ERROR;
    } else {
        ERROR |= bmi088_accel_init();
    }

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR) {
        ERROR |= BMI088_SELF_TEST_GYRO_ERROR;
    } else {
        ERROR |= bmi088_gyro_init();
    }
    return ERROR;
}

uint8_t bmi088_accel_init(void) {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_ACCEL_Write_Single_Reg(BMI088_ACC_SOFTRESET,
                                  BMI088_ACC_SOFTRESET_VALUE);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_ACCEL_Reg_Num;
         write_reg_num++) {

        BMI088_ACCEL_Write_Single_Reg(
            write_BMI088_ACCEL_Reg_Data_ERROR[write_reg_num][0],
            write_BMI088_ACCEL_Reg_Data_ERROR[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Single_Reg(
            write_BMI088_ACCEL_Reg_Data_ERROR[write_reg_num][0], res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_Reg_Data_ERROR[write_reg_num][1]) {
            return write_BMI088_ACCEL_Reg_Data_ERROR[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void) {
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro SENSOR
    BMI088_GYRO_Write_Single_Reg(BMI088_GYRO_SOFTRESET,
                                 BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_Write_GYRO_Reg_Num;
         write_reg_num++) {

        BMI088_GYRO_Write_Single_Reg(
            write_BMI088_GYRO_Reg_Data_ERROR[write_reg_num][0],
            write_BMI088_GYRO_Reg_Data_ERROR[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_GYRO_Read_Single_Reg(
            write_BMI088_GYRO_Reg_Data_ERROR[write_reg_num][0], res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_GYRO_Reg_Data_ERROR[write_reg_num][1]) {
            return write_BMI088_GYRO_Reg_Data_ERROR[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t bmi088_accel_self_test(void) {

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_ERROR[6][3] = {
        {BMI088_ACC_CONF,
         BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set,
         BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,
         BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,
         BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL,
         BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL,
         BMI088_ACC_PWR_CONF_ERROR}

    };
    //check commiunication is normal
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel SENSOR and wait for > 50ms
    BMI088_ACCEL_Write_Single_Reg(BMI088_ACC_SOFTRESET,
                                  BMI088_ACC_SOFTRESET_VALUE);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++) {

        BMI088_ACCEL_Write_Single_Reg(
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num][0],
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Single_Reg(
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num][0], res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res !=
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num][1]) {
            return write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num]
                                                              [2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++) {

        BMI088_ACCEL_Write_Single_Reg(
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num + 4][0],
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num + 4][1]);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_Read_Single_Reg(
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num + 4][0],
            res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res !=
            write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num + 4][1]) {
            return write_BMI088_ACCEL_self_test_Reg_Data_ERROR[write_reg_num +
                                                               4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_ACCEL_Read_Muli_Reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t) ((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t) ((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t) ((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_ACCEL_Write_Single_Reg(BMI088_ACC_SELF_TEST,
                                  BMI088_ACC_SELF_TEST_OFF);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_SELF_TEST, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF)) {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel SENSOR
    BMI088_ACCEL_Write_Single_Reg(BMI088_ACC_SOFTRESET,
                                  BMI088_ACC_SOFTRESET_VALUE);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) ||
        (self_test_accel[0][1] - self_test_accel[1][1] < 1365) ||
        (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_self_test(void) {
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro SENSOR
    BMI088_GYRO_Write_Single_Reg(BMI088_GYRO_SOFTRESET,
                                 BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_GYRO_Write_Single_Reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    BMI088_Delay_ms(BMI088_LONG_DELAY_TIME);

    do {

        BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_SELF_TEST, res);
        BMI088_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

void BMI088_read_gyro_who_am_i(void) {
    uint8_t buf;
    BMI088_GYRO_Read_Single_Reg(BMI088_GYRO_CHIP_ID, buf);
}

uint8_t BMI088_read_accel_who_am_i(void) {
    uint8_t buf;
    BMI088_ACCEL_Read_Single_Reg(BMI088_ACC_CHIP_ID, buf);
    return buf;
}

void BMI088_temperature_read(uint8_t* rx_buf, float* temperature) {
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t) ((rx_buf[0] << 3) | (rx_buf[1] >> 5));

    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }
    *temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088_accel_read(uint8_t* rx_buf, float accel[3], float* time) {
    int16_t bmi088_raw_temp;
    uint32_t SENSOR_time;
    bmi088_raw_temp = (int16_t) ((rx_buf[1]) << 8) | rx_buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t) ((rx_buf[3]) << 8) | rx_buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t) ((rx_buf[5]) << 8) | rx_buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    SENSOR_time = (uint32_t) ((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
    *time = SENSOR_time * 39.0625f;
}

void BMI088_gyro_read(uint8_t* rx_buf, float gyro[3]) {
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t) ((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t) ((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t) ((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

static int16_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
static uint8_t gyro_read_status = 100;
void BMI088_Read(float gyro[3], float accel[3], float* temperature) {
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_ACCEL_Read_Muli_Reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t) ((buf[1]) << 8) | buf[0];
    accel[0] =
        bmi088_raw_temp * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;
    bmi088_raw_temp = (int16_t) ((buf[3]) << 8) | buf[2];
    accel[1] =
        bmi088_raw_temp * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;
    bmi088_raw_temp = (int16_t) ((buf[5]) << 8) | buf[4];
    accel[2] =
        bmi088_raw_temp * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;

    // BMI088_GYRO_Read_Muli_Reg(BMI088_GYRO_X_L, buf, 6);
    // if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
    // bmi088_raw_temp = (int16_t) ((buf[1]) << 8) | buf[0];
    // gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
    // gyro_x = bmi088_raw_temp;
    // bmi088_raw_temp = (int16_t) ((buf[3]) << 8) | buf[2];
    // gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
    // gyro_y = bmi088_raw_temp;
    // bmi088_raw_temp = (int16_t) ((buf[5]) << 8) | buf[4];
    // gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
    // gyro_z = bmi088_raw_temp;
    // }
    uint8_t gyro_tx[7] = {
        (BMI088_GYRO_X_L | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t gyro_rx[7];

    BMI088_GYRO_NS_L();
    uint8_t status = BMI088_Gyro_RW_Byte(gyro_rx, gyro_tx, 7);
    BMI088_GYRO_NS_H();

    gyro_read_status = status;

    bmi088_raw_temp = (int16_t) ((gyro_rx[2] << 8) | gyro_rx[1]);
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
    bmi088_raw_temp = (int16_t) ((gyro_rx[4] << 8) | gyro_rx[3]);
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
    bmi088_raw_temp = (int16_t) ((gyro_rx[6] << 8) | gyro_rx[5]);
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;

    BMI088_ACCEL_Read_Muli_Reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t) ((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023) {
        bmi088_raw_temp -= 2048;
    }

    *temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t get_BMI088_SENSOR_time(void) {
    uint32_t SENSOR_time = 0;
    uint8_t buf[3];
    BMI088_ACCEL_Read_Muli_Reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    SENSOR_time = (uint32_t) ((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return SENSOR_time;
}

float get_BMI088_temperature(void) {
    uint8_t buf[2];
    float temperature;
    int16_t temperature_raw_temp;
    BMI088_ACCEL_Read_Muli_Reg(BMI088_TEMP_M, buf, 2);
    temperature_raw_temp = (int16_t) ((buf[0] << 3) | (buf[1] >> 5));
    if (temperature_raw_temp > 1023) {
        temperature_raw_temp -= 2048;
    }
    temperature =
        temperature_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    return temperature;
}

void get_BMI088_gyro(int16_t gyro[3]) {
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp;

    BMI088_GYRO_Read_Muli_Reg(BMI088_GYRO_X_L, buf, 6);

    gyro_raw_temp = (int16_t) ((buf[1]) << 8) | buf[0];
    gyro[0] = gyro_raw_temp;
    gyro_raw_temp = (int16_t) ((buf[3]) << 8) | buf[2];
    gyro[1] = gyro_raw_temp;
    gyro_raw_temp = (int16_t) ((buf[5]) << 8) | buf[4];
    gyro[2] = gyro_raw_temp;
}

void get_BMI088_accel(float accel[3]) {
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t accel_raw_temp;

    BMI088_ACCEL_Read_Muli_Reg(BMI088_ACCEL_XOUT_L, buf, 6);

    accel_raw_temp = (int16_t) ((buf[1]) << 8) | buf[0];
    accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t) ((buf[3]) << 8) | buf[2];
    accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t) ((buf[5]) << 8) | buf[4];
    accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}

static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data) {
    BMI088_Read_Write_Byte(reg);
    BMI088_Read_Write_Byte(data);
}

static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t* return_data) {
    BMI088_Read_Write_Byte(reg | 0x80);
    *return_data = BMI088_Read_Write_Byte(0x55);
}

static void BMI088_Read_Muli_Reg(uint8_t reg, uint8_t* buf, uint8_t len) {
    BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0) {
        *buf = BMI088_Read_Write_Byte(0x55);
        buf++;
        len--;
    }
}

#endif
