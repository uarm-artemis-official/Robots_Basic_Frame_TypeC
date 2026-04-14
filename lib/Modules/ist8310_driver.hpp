#ifndef __IST8310_DRIVER_HPP
#define __IST8310_DRIVER_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace ist8310_driver {
    struct SensorData {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    class IST8310 {
       private:
        MW_I2C::II2C& i2c;
        MW_RTOS::IRTOS& rtos;

       public:
        IST8310(MW_I2C::II2C& i2c_, MW_RTOS::IRTOS& rtos_)
            : i2c(i2c_), rtos(rtos_) {}

        void init(MW_I2C::II2C& i2c, MW_RTOS::IRTOS& rtos) {}

        bool read_sensor_data(SensorData& data) {}
    };
}  // namespace ist8310_driver

#endif