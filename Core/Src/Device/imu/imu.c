/*******************************************************************************
* @file           : imu.c
* @brief          : gyroscope BMI088/IST8310 read/write for attitude reading
* @restructed     : Nov, 2023
* @maintainer     : Haoran
********************************************************************************
* Copyright (c) 2023 UARM Artemis.
* All rights reserved.
*******************************************************************************/

#ifndef __IMU_C__
#define __IMU_C__

#include "imu.h"

#define ABS_F(x) (x) < 0 ? -(x) : (x)

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
  {0.0f,  1.0f, 0.0f},                    \
  {-1.0f, 0.0f, 0.0f},                 \
  {0.0f,  0.0f, 1.0f}


static bmi088_real_data_t bmi088_real_data;
static float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static float gyro_offset[3];
static float gyro_cali_offset[3];
static float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
static float accel_offset[3];
static float accel_cali_offset[3];
static float temperature = 0;

static float gyro[3], accel[3], mag[3];
//static float ins_quat[4];
//float ins_angle[3];

static void bmi088_cali_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088);

void bmi088_get_data(AhrsSensor_t *sensor, float *temp)
{
	/* read bmi088 rawa data */
    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, temp);
    /* data fusion with the offset */
    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

    /* Access the mag */
//    ist8310_read_mag(mag);

    sensor->ax = accel[0];
    sensor->ay = accel[1];
    sensor->az = accel[2];

    sensor->wx = gyro[0];
    sensor->wy = gyro[1];
    sensor->wz = -gyro[2];

//    sensor->mx = mag[0];
//    sensor->my = mag[1];
//    sensor->mz = mag[2];

    sensor->mx = 0;
    sensor->my = 0;
    sensor->mz = 0;
}

void bmi088_get_temp(float *tmp)
{
    *tmp = temperature;
}

/**
  * @brief  bmi088 init
  * @param
  * @retval error code
  */
uint8_t bmi088_device_init(void)
{
    BMI088_init();
    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperature);
    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

//    AHRS_init(ins_quat, accel, mag);
//    get_angle(ins_quat, ins_angle, ins_angle + 1, ins_angle + 2);//for ist

    return 0;
}

int ahrs_update(AhrsSensor_t *sensor, uint8_t period_ms)
{
    BMI088_Read(bmi088_real_data.gyro, bmi088_real_data.accel, &temperature);
    bmi088_cali_slove(gyro, accel, &bmi088_real_data);

    /* Access the mag */
    ist8310_read_mag(mag);

    sensor->ax = accel[0];
    sensor->ay = accel[1];
    sensor->az = accel[2];

    sensor->wx = gyro[0];
    sensor->wy = gyro[1];
    sensor->wz = gyro[2];

    sensor->mx = mag[0];
    sensor->my = mag[1];
    sensor->mz = mag[2];


//    AHRS_update(ins_quat, period_ms / 1000.0f, gyro, accel, mag);
//    get_angle(ins_quat, ins_angle, ins_angle + 1, ins_angle + 2);
//    sensor->yaw = ins_angle[0];
//    sensor->pitch = ins_angle[1];
//    sensor->roll = ins_angle[2];

    return 0;
}

/**
  * @brief  bmi088 get gyrp offset
  * @param
  * @retval error code
  */
uint8_t bmi088_set_offset(void)
{

    float gyro[3], accel[3];
    int cali_times = 100;

    for (int i = 0; i < cali_times; i++)
    {
        BMI088_Read(gyro, accel, &temperature);
        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];

        accel_offset[0] += accel[0];
        accel_offset[1] += accel[1];
        accel_offset[2] += accel[2];

        /* delay a given period */
        osDelay(3);//3
    }

    gyro_offset[0] = gyro_offset[0] / cali_times;
    gyro_offset[1] = gyro_offset[1] / cali_times;
    gyro_offset[2] = gyro_offset[2] / cali_times;

    accel_offset[0] += accel_offset[0] / cali_times;
	accel_offset[1] += accel_offset[1] / cali_times;
	accel_offset[2] += accel_offset[2] / cali_times;

    return 0;
}

uint8_t bmi088_get_offset(void)
{
    size_t read_len = 0;
    if (read_len == sizeof(gyro_offset))
    {
        /* read ok */
        return 0;
    }
    else
    {
        bmi088_set_offset();
    }

    return 0;
}

static void bmi088_cali_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] - gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] - accel_offset[i];
    }
}

#endif
