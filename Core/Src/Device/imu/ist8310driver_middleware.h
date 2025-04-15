/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       IST8310driver_middleware.c/h
  * @brief      the file provide I2C write/read function, as the middleware of IST8310.
  *             ���ļ���Ҫ�ṩI2C ��д��������ΪIST8310�������м��
  * @note       IST8310 only support I2C. IST8310ֻ֧��I2C��
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

//#include "struct_typedef.h"
#include "i2c.h"
#include "stdint.h"

extern I2C_HandleTypeDef hi2c3;

#define IST8310_IIC_ADDRESS 0x0E  //the I2C address of IST8310


/**
  * @brief          initialize ist8310 gpio.
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʼ��IST8310��GPIO
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_GPIO_init(void);

/**
  * @brief          initialize ist8310 communication interface
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��ʼ��IST8310��ͨ�Žӿ�
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_com_init(void);


/**
  * @brief          read a byte of ist8310 by i2c
  * @param[in]      register address
  * @retval         value of the register
  */
/**
  * @brief          ��ȡIST8310��һ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ַ
  * @retval         �Ĵ���ֵ
  */
extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);

/**
  * @brief          write a byte of ist8310 by i2c
  * @param[in]      register address
  * @param[in]      write value
  * @retval         value of the register
  */
/**
  * @brief          ͨ��I2Cд��һ���ֽڵ�IST8310�ļĴ�����
  * @param[in]      �Ĵ�����ַ
  * @param[in]      д��ֵ
  * @retval         none
  */
extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);

/**
  * @brief          read multiple byte of ist8310 by i2c
  * @param[in]      register start address
  * @param[out]     read buffer
  * @param[in]      Size Amount of data to be read
  * @retval         none
  */
/**
  * @brief          ��ȡIST8310�Ķ���ֽ�ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/**
  * @brief          write multiple byte of ist8310 by i2c
  * @param[in]      register address
  * @param[out]     write buffer
  * @param[in]      Size Amount of data to be sent
  * @retval         none
  */
/**
  * @brief          д�����ֽڵ�IST8310�ļĴ���ͨ��I2C
  * @param[in]      �Ĵ�����ʼ��ַ
  * @param[out]     ��ȡ������
  * @param[in]      ��ȡ�ֽ�����
  * @retval         none
  */
extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

/**
  * @brief          delay x millisecond
  * @param[in]      ms: ms millisecond
  * @retval         none
  */
/**
  * @brief          ��ʱx����
  * @param[in]      ms: ms����
  * @retval         none
  */
extern void ist8310_delay_ms(uint16_t ms);
/**
  * @brief          delay x microsecond
  * @param[in]      us: us microsecond
  * @retval         none
  */
/**
  * @brief          ��ʱx΢��
  * @param[in]      us: us΢��
  * @retval         none
  */
extern void ist8310_delay_us(uint16_t us);
/**
  * @brief          set the RSTN PIN to 1
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����RSTN����Ϊ1
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_H(void);
/**
  * @brief          set the RSTN PIN to 0
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����RSTN����Ϊ0
  * @param[in]      none
  * @retval         none
  */
extern void ist8310_RST_L(void);

#endif
