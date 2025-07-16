//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC4_H
#define ENGINEERING_USER_CONTROL_IIC4_H
/***********************************************************************************************************************************
 ***********************************************************************************************************************************
 **���ļ����ơ�  i2c_moni.h
 **������������  ģ��IICʱ��
 **              �������š�����ȫ�ֽṹ�塢����ȫ�ֺ���
 **
 **������ƽ̨��  STM32F103 + ��׼��v3.5 + keil5
 **
 **����ֲ˵����  �����޸ģ���i2c_moni.h�ļ����޸ģ��Է���IIC���߸��á����븴��
 **              ������ַ���ڸ��豸�ļ����޸�
 **
 **�����¼�¼��  2020-03-05  ����
 **              2021-05-03  �����ļ���ʽ��ע�͸�ʽ
 **
***********************************************************************************************************************************/
#include <stm32f411xe.h>
#include <stdio.h>
#include "main.h"
#include "iic.h"

// SCL
#define I2C_MONI_SCL_GPIO4      IIC_SCL4_GPIO_Port
#define I2C_MONI_SCL_PIN4       IIC_SCL4_Pin
// SDA
#define I2C_MONI_SDA_GPIO4      IIC_SDA4_GPIO_Port
#define I2C_MONI_SDA_PIN4       IIC_SDA4_Pin



int16_t getRawAngle4(void);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
