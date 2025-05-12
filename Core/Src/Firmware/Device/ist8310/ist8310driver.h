
/**
  * @file       IST8310driver.c/h
  * @brief      ist8310 is a 3-axis digital magnetometer, the file includes initialization function,
  *             read magnetic field strength data function.
  * @note       IST8310 only support I2C
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H
#include "ist8310driver_middleware.h"
#include "stdint.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40


typedef float fp32;

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

/**
  * @brief          initialize ist8310
  * @param[in]      none
  * @retval         error value
  */
/**
  * @brief          ��ʼ��IST8310
  * @param[in]      none
  * @retval         error value
  */ 
extern uint8_t ist8310_init(void);

/**
  * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve. 
  * @param[in]      status_buf:the data point from the STAT1(0x02) register of IST8310 to the DATAZL(0x08) register 
  * @param[out]     ist8310_real_data:ist8310 data struct 
  * @retval         none
  */
/**
  * @brief          ����Ѿ�ͨ��I2C��DMA��ʽ��ȡ���˴�STAT1��DATAZL�����ݣ�����ʹ������������д���
  * @param[in]      status_buf:����ָ��,��STAT1(0x02) �Ĵ����� DATAZL(0x08)�Ĵ��� 
  * @param[out]     ist8310_real_data:ist8310�����ݽṹ
  * @retval         none
  */
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);

/**
  * @brief          read mag magnetic field strength data of IST8310 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          ͨ����ȡ�ų�����
  * @param[out]     �ų�����
  * @retval         none
  */
extern void ist8310_read_mag(fp32 mag[3]);
#endif
