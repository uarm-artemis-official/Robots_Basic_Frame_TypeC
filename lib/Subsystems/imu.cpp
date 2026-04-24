#include "bmi088_driver.hpp"
#include "ist8310_driver.hpp"
#include "subsystems_classes.hpp"
#include "subsystems_defines.hpp"
#include "uarm_lib.hpp"

Imu::Imu(bmi088_driver::BMI088& bmi088_, ist8310_driver::IST8310& ist8310_,
                 MW_RTOS::IRTOS& rtos_, uint32_t sampling_rate_, float beta_,
                 const float orientation_[3][3])
    : bmi088(bmi088_),
      ist8310(ist8310_),
            rtos(rtos_),
      madgewick(sampling_rate_, beta_),
      temperature(0.0f),
      gyro {0.0f, 0.0f, 0.0f},
      accel {0.0f, 0.0f, 0.0f},
      mag {0.0f, 0.0f, 0.0f},
      orientation {
          {orientation_[0][0], orientation_[0][1], orientation_[0][2]},
          {orientation_[1][0], orientation_[1][1], orientation_[1][2]},
          {orientation_[2][0], orientation_[2][1], orientation_[2][2]}} {}

void Imu::init() {
    bmi088.init();
    ist8310.init();
}

float Imu::get_temp() {
    temperature = bmi088.get_temperature();
    return temperature;
}

void Imu::get_attitude(Attitude_t& attitude) {
    constexpr bool use_magnetometer = false;
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
    rtos.critical_section_enter();
    bmi088_driver::BMI088Data bmi088_raw_data;
    bmi088.get_all_data(bmi088_raw_data);

    float accel_data[3] = {bmi088_raw_data.acceleration.x,
                           bmi088_raw_data.acceleration.y,
                           bmi088_raw_data.acceleration.z};
    float gyro_data[3] = {bmi088_raw_data.gyro.x, bmi088_raw_data.gyro.y,
                          bmi088_raw_data.gyro.z};
    temperature = bmi088_raw_data.temperature;

    adjust_data(accel, accel_data, accel_bias, accel_scale);
    adjust_data(gyro, gyro_data, gyro_bias, gyro_scale);

    if (read_mag) {
        ist8310_driver::MagnetometerData mag_data;
        const bool is_mag_data_valid = ist8310.read_magnetometer_data(mag_data);
        if (is_mag_data_valid) {
            mag[0] = mag_data.x;
            mag[1] = mag_data.y;
            mag[2] = mag_data.z;
        } else {
            mag[0] = 0.0f;
            mag[1] = 0.0f;
            mag[2] = 0.0f;
        }
    } else {
        mag[0] = 0.0f;
        mag[1] = 0.0f;
        mag[2] = 0.0f;
    }

    get_sensor_data(sensor);
    rtos.critical_section_exit();
}

void Imu::adjust_data(float output[3], float data[3], const float bias[3],
                      const float scale[3][3]) {
    float tmp[3];
    float tmp_output[3];

    for (size_t i = 0; i < 3; i++) {
        tmp[i] = data[i] - bias[i];
    }

    for (size_t i = 0; i < 3; i++) {
        tmp_output[i] =
            scale[i][0] * tmp[0] + scale[i][1] * tmp[1] + scale[i][2] * tmp[2];
    }

    for (size_t i = 0; i < 3; i++) {
        output[i] = orientation[i][0] * tmp_output[0] +
                    orientation[i][1] * tmp_output[1] +
                    orientation[i][2] * tmp_output[2];
    }
}

void Imu::set_heat_pwm(uint16_t pwm) {
    ASSERT(pwm <= 4000,
           "Duty cycle cannot be set greater than timer 10's counter.");
    bmi088.set_heat_pwm_duty_cycle(pwm);
}
