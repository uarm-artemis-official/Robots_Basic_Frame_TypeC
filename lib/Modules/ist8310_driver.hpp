#ifndef __IST8310_DRIVER_HPP
#define __IST8310_DRIVER_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace ist8310_driver {
    struct MagnetometerData {
        float x;
        float y;
        float z;
    };

    class IST8310 {
       private:
        static constexpr uint8_t DEVICE_ADDRESS = 0x0E;
        static constexpr uint16_t MEM_ADDRESS_SIZE_8BIT = 1;
        static constexpr uint32_t I2C_TIMEOUT_MS = 10;
        static constexpr uint8_t WHO_AM_I_REGISTER = 0x00;
        static constexpr uint8_t WHO_AM_I_VALUE = 0x10;
        static constexpr uint32_t RESET_DELAY_MS = 50;
        // Original C implementation waits 150 us. RTOS abstraction exposes ms delay.
        static constexpr uint32_t REGISTER_SETTLE_DELAY_MS = 1;
        static constexpr uint8_t DATA_START_REGISTER = 0x03;
        static constexpr float SENSITIVITY = 0.3f;

        static constexpr MW_I2C::Periperhal I2C_BUS = MW_I2C::Periperhal::I2C_3;
        static constexpr MW_GPIO::Port RSTN_PORT = MW_GPIO::Port::PORT_G;
        static constexpr MW_GPIO::Pin RSTN_PIN = MW_GPIO::Pin::PIN_6;

        static constexpr std::array<std::array<uint8_t, 3>, 4> INIT_REGISTERS {{
            {{0x0B, 0x08, 0x01}},
            {{0x41, 0x09, 0x02}},
            {{0x42, 0xC0, 0x03}},
            {{0x0A, 0x0B, 0x04}},
        }};

        MW_I2C::II2C& i2c;
        MW_RTOS::IRTOS& rtos;
        MW_GPIO::IGPIO& gpio;

       public:
        IST8310(MW_I2C::II2C& i2c_, MW_RTOS::IRTOS& rtos_,
                MW_GPIO::IGPIO& gpio_)
            : i2c(i2c_), rtos(rtos_), gpio(gpio_) {}

        void init(MW_I2C::II2C& i2c, MW_RTOS::IRTOS& rtos) {
            gpio.write_pin(RSTN_PORT, RSTN_PIN, MW_GPIO::State::LOW);
            rtos.delay(RESET_DELAY_MS);
            gpio.write_pin(RSTN_PORT, RSTN_PIN, MW_GPIO::State::HIGH);
            rtos.delay(RESET_DELAY_MS);

            uint8_t who_am_i = 0;
            i2c.mem_read(I2C_BUS, DEVICE_ADDRESS, WHO_AM_I_REGISTER,
                         MEM_ADDRESS_SIZE_8BIT, &who_am_i, 1, I2C_TIMEOUT_MS);
            ASSERT(who_am_i == WHO_AM_I_VALUE, "IST8310 sensor not detected.");

            for (const auto& reg_data : INIT_REGISTERS) {
                i2c.mem_write(I2C_BUS, DEVICE_ADDRESS, reg_data[0],
                              MEM_ADDRESS_SIZE_8BIT, &reg_data[1], 1,
                              I2C_TIMEOUT_MS);
                rtos.delay(REGISTER_SETTLE_DELAY_MS);

                uint8_t reg_value = 0;
                i2c.mem_read(I2C_BUS, DEVICE_ADDRESS, reg_data[0],
                             MEM_ADDRESS_SIZE_8BIT, &reg_value, 1,
                             I2C_TIMEOUT_MS);
                rtos.delay(REGISTER_SETTLE_DELAY_MS);

                ASSERT(reg_value == reg_data[1],
                       "IST8310 register verification failed.");
            }
        }

        bool read_magnetometer_data(MagnetometerData& data) {
            uint8_t buf[6] = {0};
            i2c.mem_read(I2C_BUS, DEVICE_ADDRESS, DATA_START_REGISTER,
                         MEM_ADDRESS_SIZE_8BIT, buf, sizeof(buf),
                         I2C_TIMEOUT_MS);

            const int16_t raw_x = static_cast<int16_t>((buf[1] << 8) | buf[0]);
            const int16_t raw_y = static_cast<int16_t>((buf[3] << 8) | buf[2]);
            const int16_t raw_z = static_cast<int16_t>((buf[5] << 8) | buf[4]);

            data.x = static_cast<float>(raw_x) * SENSITIVITY;
            data.y = static_cast<float>(raw_y) * SENSITIVITY;
            data.z = static_cast<float>(raw_z) * SENSITIVITY;
            return true;
        }
    };
}  // namespace ist8310_driver

#endif