#ifndef __IMU_H
#define __IMU_H

#include "madgewick.h"  // TODO: move to attitude_types.h eventually.
#include "subsystems_interfaces.h"
#include "subsystems_types.h"

class Imu : public IImu {
   private:
    Madgewick_Filter madgewick;
    float temperature;
    float gyro[3], accel[3], mag[3];

    const float accel_bias[3] = {0.076091, -0.056203, 0.049820};
    const float accel_scale[3][3] = {{1.004290, 0.002457, 0.000033},
                                     {0.002457, 1.002675, 0.002101},
                                     {0.000033, 0.002101, 0.994819}};
    const float gyro_bias[3] = {0.00127898, 0.00048873, 0.00255299};
    const float gyro_scale[3][3] = {
        {1.0, 0, 0},
        {0.0, 1.0, 0},
        {0.0, 0, 1.0},
    };

   public:
    static void adjust_data(float output[3], float data[3], const float bias[3],
                            const float scale[3][3]);
    Imu(uint32_t sampling_rate_, float beta_);
    void init() override;
    float get_temp() override;
    void get_attitude(Attitude_t& attitude) override;
    void get_sensor_data(AhrsSensor_t& sensor) override;
    void set_heat_pwm(uint16_t duty_cycle) override;
    void gather_sensor_data(AhrsSensor_t& sensor, bool read_mag) override;
};

#endif