#ifndef __IMU_H
#define __IMU_H

#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class Imu : public IImu {
   private:
    float temperature;
    float gyro_offset[3], accel_offset[3];
    float gyro_cali_offset[3], accel_cali_offset[3];
    float gyro[3], accel[3], mag[3];

   public:
    static constexpr const float gyro_scale_factor[3][3] = {
        BMI088_BOARD_INSTALL_SPIN_MATRIX};
    static constexpr const float accel_scale_factor[3][3] = {
        BMI088_BOARD_INSTALL_SPIN_MATRIX};

    void init() override;
    float get_temp() override;
    void get_attitude(Attitude_t* attitude) override;
    void set_offset() override;
    void set_heat_pwm(uint16_t duty_cycle) override;
    void set_cali_slove() override;
    void ahrs_update(AhrsSensor_t* sensor, bool read_mag) override;
};

#endif