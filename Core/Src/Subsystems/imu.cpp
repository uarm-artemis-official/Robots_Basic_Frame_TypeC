#include "imu.h"
#include "subsystems_defines.h"

#include "ahrs.h"
#include "bmi088_driver.h"
#include "ist8310driver.h"
#include "uarm_lib.h"
#include "uarm_os.h"

#include "tim.h"

void Imu::init() {
    memset(gyro_offset, 0, sizeof(float) * 3);
    memset(accel_offset, 0, sizeof(float) * 3);
    BMI088_init();
    ist8310_init();
    set_cali_slove();
}

float Imu::get_temp() {
    BMI088_Read(gyro, accel, &temperature);
    return temperature;
}

void Imu::get_attitude(Attitude_t* attitude) {
    AhrsSensor_t sensor;
    float temp;
    ahrs_update(&sensor, false);
    madgwick_ahrs_updateIMU(&sensor, attitude);
    // madgwick_ahrs_update(&sensor, attitude);
}

void Imu::ahrs_update(AhrsSensor_t* sensor, bool read_mag) {
    set_cali_slove();

    /* Access the mag */

    sensor->ax = accel[0];
    sensor->ay = accel[1];
    sensor->az = accel[2];

    sensor->wx = gyro[0];
    sensor->wy = gyro[1];
    sensor->wz = gyro[2];

    if (read_mag) {
        ist8310_read_mag(mag);
        sensor->mx = mag[0];
        sensor->my = mag[1];
        sensor->mz = mag[2];
    } else {
        sensor->mx = 0;
        sensor->my = 0;
        sensor->mz = 0;
    }

    //    AHRS_update(ins_quat, period_ms / 1000.0f, gyro, accel, mag);
    //    get_angle(ins_quat, ins_angle, ins_angle + 1, ins_angle + 2);
    //    sensor->yaw = ins_angle[0];
    //    sensor->pitch = ins_angle[1];
    //    sensor->roll = ins_angle[2];
}

void Imu::set_offset() {
    int cali_times = 100;

    for (int i = 0; i < cali_times; i++) {
        BMI088_Read(gyro, accel, &temperature);
        gyro_offset[0] += gyro[0];
        gyro_offset[1] += gyro[1];
        gyro_offset[2] += gyro[2];
        osDelay(3);
    }

    gyro_offset[0] = gyro_offset[0] / cali_times;
    gyro_offset[1] = gyro_offset[1] / cali_times;
    gyro_offset[2] = gyro_offset[2] / cali_times;
}

void Imu::set_cali_slove() {
    bmi088_real_data_t bmi088_raw_data;
    BMI088_Read(bmi088_raw_data.gyro, bmi088_raw_data.accel, &temperature);
    for (uint8_t i = 0; i < 3; i++) {
        gyro[i] = bmi088_raw_data.gyro[0] * Imu::gyro_scale_factor[i][0] +
                  bmi088_raw_data.gyro[1] * Imu::gyro_scale_factor[i][1] +
                  bmi088_raw_data.gyro[2] * Imu::gyro_scale_factor[i][2] -
                  gyro_offset[i];
        accel[i] = bmi088_raw_data.accel[0] * Imu::accel_scale_factor[i][0] +
                   bmi088_raw_data.accel[1] * Imu::accel_scale_factor[i][1] +
                   bmi088_raw_data.accel[2] * Imu::accel_scale_factor[i][2] -
                   accel_offset[i];
    }
}

void Imu::set_heat_pwm(uint16_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}