#include "imu.h"
#include "ahrs.h"
#include "bmi088_driver.h"
#include "ist8310driver.h"
#include "subsystems_defines.h"
#include "uarm_lib.h"
#include "uarm_os.h"

Imu::Imu(uint32_t sampling_rate_, float beta_)
    : madgewick(sampling_rate_, beta_) {}

void Imu::init() {
    BMI088_init();
    ist8310_init();
}

float Imu::get_temp() {
    BMI088_Read(gyro, accel, &temperature);
    return temperature;
}

void Imu::get_attitude(Attitude_t& attitude) {
    constexpr bool use_magnetometer = true;
    AhrsSensor_t sensor;

    gather_sensor_data(sensor, use_magnetometer);
    madgewick.calc_marg(sensor.ax, sensor.ay, sensor.az, sensor.wx, sensor.wy,
                        sensor.wz, sensor.mx, sensor.my, sensor.mz);
    madgewick.get_attitude(attitude);
}

void Imu::get_sensor_data(AhrsSensor_t& sensor) {
    sensor.ax = accel[0];
    sensor.ay = accel[1];
    sensor.az = accel[2];

    sensor.wx = gyro[0];
    sensor.wy = gyro[1];
    sensor.wz = gyro[2];

    sensor.mx = mag[0];
    sensor.my = mag[1];
    sensor.mz = mag[2];
}

void Imu::gather_sensor_data(AhrsSensor_t& sensor, bool read_mag) {
    taskENTER_CRITICAL();
    bmi088_real_data_t bmi088_raw_data;
    BMI088_Read(bmi088_raw_data.gyro, bmi088_raw_data.accel, &temperature);

    Imu::adjust_data(accel, bmi088_raw_data.accel, accel_bias, accel_scale);
    Imu::adjust_data(gyro, bmi088_raw_data.gyro, gyro_bias, gyro_scale);

    if (read_mag) {
        ist8310_read_mag(mag);
    } else {
        mag[0] = 0.0f;
        mag[1] = 0.0f;
        mag[2] = 0.0f;
    }

    get_sensor_data(sensor);
    taskEXIT_CRITICAL();
}

void Imu::adjust_data(float output[3], float data[3], const float bias[3],
                      const float scale[3][3]) {
    float tmp[3];
    for (int i = 0; i < 3; i++) {
        tmp[i] = data[i] - bias[i];
    }

    for (int i = 0; i < 3; i++) {
        output[i] =
            scale[i][0] * tmp[0] + scale[i][1] * tmp[1] + scale[i][2] * tmp[2];
    }
}

void Imu::set_heat_pwm(uint16_t pwm) {
    ASSERT(pwm <= 4000,
           "Duty cycle cannot be set greater than timer 10's counter.");
    BMI088_Set_PWM_Duty_Cycle(pwm);
}
