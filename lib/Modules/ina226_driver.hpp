#ifndef __INA226_DRIVER_HPP
#define __INA226_DRIVER_HPP

#include "middleware_interfaces.hpp"
#include "uarm_lib.hpp"


//Swap LSB
#define SWAP_ENDIAN(x) ((((x) & 0xff) << 8) | (((x) & 0xff00) >> 8))


namespace ina226_driver {

    class INA226 {

        private:

            static constexpr uint8_t CONFIG_REG = 0x00;
            static constexpr uint8_t SHUNT_VOLTAGE_REG = 0x01;
            static constexpr uint8_t BUS_VOLTAGE_REG = 0x02;
            static constexpr uint8_t POWER_REG = 0x03;
            static constexpr uint8_t CURRENT_REG = 0x04;
            static constexpr uint8_t CALIBRATION_REG = 0x05;
            static constexpr uint8_t MASK_EN_REG = 0x06;
            static constexpr uint8_t ALERT_LIMIT_REG = 0x07;
            static constexpr uint8_t MANUFACTURE_ID_REG 0xFE;
            static constexpr uint8_t DIE_ID_REG = 0xFF;
            static constexpr uint16_t DEFAULT_DIE_ID = 0x2260;

            static constexpr uint8_t AVG_MODE = 0b000;              //Averaging over 1 sample (aka no averaging)
            static constexpr uint8_t BUS_CONV_TIME = 0b100;         //1.1ms
            static constexpr uint8_t SHUNT_CONV_TIME = 0b100;       //1.1ms
            static constexpr uint8_t MODE = 0b111;                  //Continually measure shunt and bus
            static constexpr uint16_t CALIBRATION_VAL = 8533;       //Found expirementally - may have to update 
            static constexpr uint16_t RESERVED_BIT = 0b0100000000000000;    //Need to keep Reserved Bit to a 1


            static constexpr uint16_t CONFIG_VAL =  RESERVED_BIT | (AVG_MODE << 9) | (BUS_CONV_TIME << 6) | (SHUNT_CONV_TIME << 3) | AVG_MODE;
            


        static constexpr std::array<std::array<uint8_t, 3>, 2> INIT_REGISTERS {{


            {{CONFIG_REG, SWAP_ENDIAN(CONFIG_VAL), 0x01}},
            {{CALIBRATION_REG, SWAP_ENDIAN(CALIBRATION_VAL), 0x02}},
        }};


            static constexpr MW_I2C::Periperhal I2C_BUS = MW_I2C::Periperhal::I2C_3;

            MW_I2C::II2C& i2c;
            MW_RTOS::IRTOS& rtos;
            uint16_t device_address;


        public:
            INA226(MW_I2C::II2C& i2c_,  MW_RTOS::IRTOS& rtos_, uint16_t device_address_) : i2c(i2c_), rtos(rtos_) device_address(device_address_) {}

            void init() {

                uint16_t device_id = 0;
                i2c.mem_read(I2C_BUS, device_address, DIE_ID_REG, MEM_ADDRESS_SIZE_16BIT, &device_id, 2, I2C_TIMEOUT_MS);
                ASSERT(device_id == DEFAULT_DIE_ID, "INA226 sensor not detected.");


                for (const auto& reg_data : INIT_REGISTERS){
                    i2c.mem_write(I2C_BUS, device_address, reg_data[0],
                              MEM_ADDRESS_SIZE_16BIT, &reg_data[1], 2,
                              I2C_TIMEOUT_MS);
                    rtos.delay(REGISTER_SETTLE_DELAY_MS);

                    uint16_t reg_value = 0;
                    i2c.mem_read(I2C_BUS, device_address, reg_data[0],
                                MEM_ADDRESS_SIZE_16BIT, &reg_value, 1,
                                I2C_TIMEOUT_MS);
                    rtos.delay(REGISTER_SETTLE_DELAY_MS);

                    ASSERT(reg_value == reg_data[1],
                        "INA226 register verification failed.");
                }
    
            }


            bool read_current_value_in_mA(int16_t& current){
                uint16_t result = 0;
                i2c.mem_read(I2C_BUS, device_address, CURRENT_REG,
                    MEM_ADDRESS_SIZE_16BIT, &result, sizeof(uint16_t),
                    I2C_TIMEOUT_MS);

                //LSB is 0.25mA (from Default Shunt Calibration Value)
                result = SWAP_ENDIAN(result);
                result = (result*25)/100;
                current = result;
                return true;

            }

    };

}




#endif