/**
****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       IST8310middleware.c/h
  * @brief      IST8310�������м�㣬���IST8310��IICͨ��,��Ϊ����MPU6500��SPIͨ��
  *             �������õ���ͨ��mpu6500��IIC_SLV0��ɶ�ȡ��IIC_SLV4���д�롣
  * @note       IST8310ֻ֧��IIC��ȡ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#ifndef IST8310DRIVER_MIDDLEWARE_H
#define IST8310DRIVER_MIDDLEWARE_H

#include "main.h"

#define IST8310_IIC_ADDRESS (0x0E << 1)
#define IST8310_IIC_READ_MSB (0x80)

#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG

extern void ist8310_GPIO_init(void);

extern void ist8310_com_init(void);

extern uint8_t ist8310_IIC_read_single_reg(uint8_t reg);

extern void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);

extern void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

extern void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);

extern void ist8310_delay_ms(uint16_t ms);

extern void ist8310_delay_us(uint16_t us);

extern void ist8310_RST_H(void);

extern void ist8310_RST_L(void);
#endif
