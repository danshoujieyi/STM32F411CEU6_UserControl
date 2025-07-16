//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC7_H
#define ENGINEERING_USER_CONTROL_IIC7_H
/***********************************************************************************************************************************
 ***********************************************************************************************************************************
 **【文件名称】  i2c_moni.h
 **【功能描述】  模拟IIC时序
 **              定义引脚、定义全局结构体、声明全局函数
 **
 **【适用平台】  STM32F103 + 标准库v3.5 + keil5
 **
 **【移植说明】  引脚修改：在i2c_moni.h文件中修改，以方便IIC总线复用、代码复用
 **              器件地址：在各设备文件中修改
 **
 **【更新记录】  2020-03-05  创建
 **              2021-05-03  完善文件格式、注释格式
 **
***********************************************************************************************************************************/
#include <stm32f411xe.h>
#include <stdio.h>
//#include "main.h"
#include "iic.h"

// SCL
#define I2C_MONI_SCL_GPIO7      IIC_SCL1_GPIO_Port
#define I2C_MONI_SCL_PIN7       IIC_SCL1_Pin
// SDA
#define I2C_MONI_SDA_GPIO7      IIC_SDA1_GPIO_Port
#define I2C_MONI_SDA_PIN7       IIC_SDA1_Pin




int16_t getRawAngle7(void);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
