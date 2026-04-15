#ifndef __BMI088_DRIVER_HPP
#define __BMI088_DRIVER_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"

namespace bmi088_driver {
    struct GyroData {
        float x;
        float y;
        float z;
    };

    struct AccelerometerData {
        float x;
        float y;
        float z;
    };

    struct BMI088Data {
        AccelerometerData acceleration;
        GyroData gyro;
        float temperature;
    };

    class BMI088 {
       private:
        static constexpr MW_SPI::Peripheral SPI_BUS = MW_SPI::Peripheral::SPI_1;

        static constexpr MW_GPIO::Port ACCEL_CS_PORT = MW_GPIO::Port::PORT_A;
        static constexpr MW_GPIO::Pin ACCEL_CS_PIN = MW_GPIO::Pin::PIN_4;
        static constexpr MW_GPIO::Port GYRO_CS_PORT = MW_GPIO::Port::PORT_B;
        static constexpr MW_GPIO::Pin GYRO_CS_PIN = MW_GPIO::Pin::PIN_0;

        static constexpr uint8_t BMI088_ACCEL_XOUT_L = 0x12;
        static constexpr uint8_t BMI088_GYRO_X_L = 0x02;
        static constexpr uint8_t BMI088_TEMP_M = 0x22;
        static constexpr uint8_t BMI088_ACC_CHIP_ID = 0x00;
        static constexpr uint8_t BMI088_ACC_CHIP_ID_VALUE = 0x1E;
        static constexpr uint8_t BMI088_ACC_SOFTRESET = 0x7E;
        static constexpr uint8_t BMI088_ACC_SOFTRESET_VALUE = 0xB6;
        static constexpr uint8_t BMI088_ACC_PWR_CTRL = 0x7D;
        static constexpr uint8_t BMI088_ACC_ENABLE_ACC_ON = 0x04;
        static constexpr uint8_t BMI088_ACC_PWR_CONF = 0x7C;
        static constexpr uint8_t BMI088_ACC_PWR_ACTIVE_MODE = 0x00;
        static constexpr uint8_t BMI088_ACC_CONF = 0x40;
        static constexpr uint8_t BMI088_ACC_NORMAL = 0x20;
        static constexpr uint8_t BMI088_ACC_800_HZ = 0x0B;
        static constexpr uint8_t BMI088_ACC_CONF_MUST_SET = 0x80;
        static constexpr uint8_t BMI088_ACC_RANGE = 0x41;
        static constexpr uint8_t BMI088_ACC_RANGE_3G = 0x00;
        static constexpr uint8_t BMI088_INT1_IO_CTRL = 0x53;
        static constexpr uint8_t BMI088_ACC_INT1_IO_ENABLE = 0x08;
        static constexpr uint8_t BMI088_ACC_INT1_GPIO_PP = 0x00;
        static constexpr uint8_t BMI088_ACC_INT1_GPIO_LOW = 0x00;
        static constexpr uint8_t BMI088_INT_MAP_DATA = 0x58;
        static constexpr uint8_t BMI088_ACC_INT1_DRDY_INTERRUPT = 0x04;

        static constexpr uint8_t BMI088_GYRO_CHIP_ID = 0x00;
        static constexpr uint8_t BMI088_GYRO_CHIP_ID_VALUE = 0x0F;
        static constexpr uint8_t BMI088_GYRO_SOFTRESET = 0x14;
        static constexpr uint8_t BMI088_GYRO_SOFTRESET_VALUE = 0xB6;
        static constexpr uint8_t BMI088_GYRO_RANGE = 0x0F;
        static constexpr uint8_t BMI088_GYRO_2000 = 0x00;
        static constexpr uint8_t BMI088_GYRO_BANDWIDTH = 0x10;
        static constexpr uint8_t BMI088_GYRO_1000_116_HZ = 0x02;
        static constexpr uint8_t BMI088_GYRO_BANDWIDTH_MUST_SET = 0x80;
        static constexpr uint8_t BMI088_GYRO_LPM1 = 0x11;
        static constexpr uint8_t BMI088_GYRO_NORMAL_MODE = 0x00;
        static constexpr uint8_t BMI088_GYRO_CTRL = 0x15;
        static constexpr uint8_t BMI088_DRDY_ON = 0x80;
        static constexpr uint8_t BMI088_GYRO_INT3_INT4_IO_CONF = 0x16;
        static constexpr uint8_t BMI088_GYRO_INT3_GPIO_PP = 0x00;
        static constexpr uint8_t BMI088_GYRO_INT3_GPIO_LOW = 0x00;
        static constexpr uint8_t BMI088_GYRO_INT3_INT4_IO_MAP = 0x18;
        static constexpr uint8_t BMI088_GYRO_DRDY_IO_INT3 = 0x01;

        static constexpr float BMI088_ACCEL_SEN = 0.000091552734375f;
        static constexpr float BMI088_GYRO_SEN = 0.06103515625f;
        static constexpr float DEGREES_TO_RADIANS = 0.0174532925199f;
        static constexpr float G_TO_METERS_PER_SECOND_SQUARED = 9.8115861f;
        static constexpr float BMI088_TEMP_FACTOR = 0.125f;
        static constexpr float BMI088_TEMP_OFFSET = 23.0f;
        // Legacy C driver waits 150 us between operations; RTOS interface is ms.
        static constexpr uint32_t SENSOR_WAIT_MS = 1;
        static constexpr uint32_t SENSOR_RESET_WAIT_MS = 80;
        static constexpr uint32_t SPI_TIMEOUT_MS = 1000;

        MW_SPI::ISPI& spi;
        MW_RTOS::IRTOS& rtos;
        MW_GPIO::IGPIO& gpio;
        MW_TIM::IPWM& pwm;

        enum class Sensor { ACCELEROMETER, GYRO };

        void set_chip_select(Sensor sensor, MW_GPIO::State state) {
            switch (sensor) {
                case Sensor::ACCELEROMETER:
                    gpio.write_pin(ACCEL_CS_PORT, ACCEL_CS_PIN, state);
                    return;
                case Sensor::GYRO:
                    gpio.write_pin(GYRO_CS_PORT, GYRO_CS_PIN, state);
                    return;
                default:
                    ASSERT(false,
                           "Invalid sensor type for BMI088 chip select.");
            }
        }

        uint8_t read_write_byte(uint8_t tx) {
            uint8_t rx = 0;
            const bool ok =
                spi.transmit_receive(SPI_BUS, &tx, &rx, 1, SPI_TIMEOUT_MS);
            ASSERT(ok, "BMI088 SPI transfer failed.");
            return rx;
        }

        void read_multi_register(Sensor sensor, uint8_t reg, uint8_t* buf,
                                 size_t len) {
            set_chip_select(sensor, MW_GPIO::State::LOW);
            (void) read_write_byte(static_cast<uint8_t>(reg | 0x80));
            for (size_t i = 0; i < len; i++) {
                buf[i] = read_write_byte(0x55);
            }
            set_chip_select(sensor, MW_GPIO::State::HIGH);
        }

        void write_single_register(Sensor sensor, uint8_t reg, uint8_t value) {
            set_chip_select(sensor, MW_GPIO::State::LOW);
            (void) read_write_byte(reg);
            (void) read_write_byte(value);
            set_chip_select(sensor, MW_GPIO::State::HIGH);
        }

        uint8_t read_single_register(Sensor sensor, uint8_t reg) {
            set_chip_select(sensor, MW_GPIO::State::LOW);
            (void) read_write_byte(static_cast<uint8_t>(reg | 0x80));
            const uint8_t value = read_write_byte(0x55);
            set_chip_select(sensor, MW_GPIO::State::HIGH);
            return value;
        }

       public:
        BMI088(MW_SPI::ISPI& spi_, MW_RTOS::IRTOS& rtos_, MW_GPIO::IGPIO& gpio_,
               MW_TIM::IPWM& pwm_)
            : spi(spi_), rtos(rtos_), gpio(gpio_), pwm(pwm_) {}

        bool accel_init() {
            uint8_t res =
                read_single_register(Sensor::ACCELEROMETER, BMI088_ACC_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            res =
                read_single_register(Sensor::ACCELEROMETER, BMI088_ACC_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);

            write_single_register(Sensor::ACCELEROMETER, BMI088_ACC_SOFTRESET,
                                  BMI088_ACC_SOFTRESET_VALUE);
            rtos.delay(SENSOR_RESET_WAIT_MS);

            res =
                read_single_register(Sensor::ACCELEROMETER, BMI088_ACC_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            res =
                read_single_register(Sensor::ACCELEROMETER, BMI088_ACC_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            if (res != BMI088_ACC_CHIP_ID_VALUE) {
                return false;
            }

            const uint8_t accel_config[][2] = {
                {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON},
                {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE},
                {BMI088_ACC_CONF,
                 static_cast<uint8_t>(BMI088_ACC_NORMAL | BMI088_ACC_800_HZ |
                                      BMI088_ACC_CONF_MUST_SET)},
                {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G},
                {BMI088_INT1_IO_CTRL,
                 static_cast<uint8_t>(BMI088_ACC_INT1_IO_ENABLE |
                                      BMI088_ACC_INT1_GPIO_PP |
                                      BMI088_ACC_INT1_GPIO_LOW)},
                {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT},
            };

            for (size_t i = 0;
                 i < (sizeof(accel_config) / sizeof(accel_config[0])); i++) {
                write_single_register(Sensor::ACCELEROMETER, accel_config[i][0],
                                      accel_config[i][1]);
                rtos.delay(SENSOR_WAIT_MS);

                res = read_single_register(Sensor::ACCELEROMETER,
                                           accel_config[i][0]);
                rtos.delay(SENSOR_WAIT_MS);
                if (res != accel_config[i][1]) {
                    return false;
                }
            }

            return true;
        }

        bool gyro_init() {
            uint8_t res =
                read_single_register(Sensor::GYRO, BMI088_GYRO_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            res = read_single_register(Sensor::GYRO, BMI088_GYRO_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);

            write_single_register(Sensor::GYRO, BMI088_GYRO_SOFTRESET,
                                  BMI088_GYRO_SOFTRESET_VALUE);
            rtos.delay(SENSOR_RESET_WAIT_MS);

            res = read_single_register(Sensor::GYRO, BMI088_GYRO_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            res = read_single_register(Sensor::GYRO, BMI088_GYRO_CHIP_ID);
            rtos.delay(SENSOR_WAIT_MS);
            if (res != BMI088_GYRO_CHIP_ID_VALUE) {
                return false;
            }

            const uint8_t gyro_config[][2] = {
                {BMI088_GYRO_RANGE, BMI088_GYRO_2000},
                {BMI088_GYRO_BANDWIDTH,
                 static_cast<uint8_t>(BMI088_GYRO_1000_116_HZ |
                                      BMI088_GYRO_BANDWIDTH_MUST_SET)},
                {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE},
                {BMI088_GYRO_CTRL, BMI088_DRDY_ON},
                {BMI088_GYRO_INT3_INT4_IO_CONF,
                 static_cast<uint8_t>(BMI088_GYRO_INT3_GPIO_PP |
                                      BMI088_GYRO_INT3_GPIO_LOW)},
                {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3},
            };

            for (size_t i = 0;
                 i < (sizeof(gyro_config) / sizeof(gyro_config[0])); i++) {
                write_single_register(Sensor::GYRO, gyro_config[i][0],
                                      gyro_config[i][1]);
                rtos.delay(SENSOR_WAIT_MS);

                res = read_single_register(Sensor::GYRO, gyro_config[i][0]);
                rtos.delay(SENSOR_WAIT_MS);
                if (res != gyro_config[i][1]) {
                    return false;
                }
            }

            return true;
        }

        void init() {
            set_chip_select(Sensor::ACCELEROMETER, MW_GPIO::State::HIGH);
            set_chip_select(Sensor::GYRO, MW_GPIO::State::HIGH);

            ASSERT(accel_init(), "BMI088 accelerometer initialization failed.");
            ASSERT(gyro_init(), "BMI088 gyroscope initialization failed.");
        }

        void get_gyro(GyroData& gyro) {
            uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
            read_multi_register(Sensor::GYRO, BMI088_GYRO_X_L, buf,
                                sizeof(buf));

            int16_t raw = static_cast<int16_t>((buf[1] << 8) | buf[0]);
            gyro.x = raw * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
            raw = static_cast<int16_t>((buf[3] << 8) | buf[2]);
            gyro.y = raw * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
            raw = static_cast<int16_t>((buf[5] << 8) | buf[4]);
            gyro.z = raw * BMI088_GYRO_SEN * DEGREES_TO_RADIANS;
        }

        float get_temperature() {
            uint8_t buf[2] = {0, 0};
            read_multi_register(Sensor::ACCELEROMETER, BMI088_TEMP_M, buf,
                                sizeof(buf));

            int16_t temperature_raw =
                static_cast<int16_t>((buf[0] << 3) | (buf[1] >> 5));
            if (temperature_raw > 1023) {
                temperature_raw -= 2048;
            }

            return temperature_raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        }

        void get_acceleration(AccelerometerData& accel) {
            uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
            int16_t raw = 0;

            read_multi_register(Sensor::ACCELEROMETER, BMI088_ACCEL_XOUT_L, buf,
                                sizeof(buf));

            raw = static_cast<int16_t>((buf[1] << 8) | buf[0]);
            accel.x = raw * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;
            raw = static_cast<int16_t>((buf[3] << 8) | buf[2]);
            accel.y = raw * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;
            raw = static_cast<int16_t>((buf[5] << 8) | buf[4]);
            accel.z = raw * BMI088_ACCEL_SEN * G_TO_METERS_PER_SECOND_SQUARED;
        }

        void get_all_data(BMI088Data& data) {
            get_acceleration(data.acceleration);
            get_gyro(data.gyro);
            data.temperature = get_temperature();
        }
    };
}  // namespace bmi088_driver

#endif