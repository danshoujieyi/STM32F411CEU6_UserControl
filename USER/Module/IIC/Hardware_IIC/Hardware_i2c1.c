//
// Created by ���ٿ� on 2025/4/5.
//

#include "main.h"
#include "arm_math.h"
#include "i2c.h"
#include "Hardware_i2c1.h"
#include "tim_delay.h"
/************************************************
 ** �ӻ���ַ_�궨��
 ***********************************************/
#define _ams5600_Address 0x36
#define _raw_ang_hi 0x0c

/*****************************************************************************
*��  ����Read_Encoder_Angle1��5��6
*��  �ܣ���ָ���ӻ��豸����ȡָ����ַ�Ķ��ֵ�� ��λ���ֽ�
*��  ������
*����ֵ��angle
*��  ע��д����ȡASM5600��ֵ
*****************************************************************************/


int16_t Read_Encoder_Angle4(void)
{
    int angle4=0;
    uint8_t data[2]={0,0};
    delayUs(20);
    HAL_I2C_Mem_Read(&hi2c3,_ams5600_Address<<1,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,data,2,50);
    angle4 = (data[0] << 8 | data[1]);
    return angle4 ;
}

int16_t Read_Encoder_Angle5(void)
{
    int angle5=0;
    uint8_t data[2]={0,0};
    delayUs(20);
    HAL_I2C_Mem_Read(&hi2c2,_ams5600_Address << 1,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,data,2,50);
    angle5 = (data[0] << 8 | data[1]);
    return angle5 ;
}

int16_t Read_Encoder_Angle6(void)
{
    int angle6=0;
    uint8_t data[2]={0,0};
    delayUs(20);
    HAL_I2C_Mem_Read(&hi2c1,_ams5600_Address << 1,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,data,2,50);
    angle6 = (data[0] << 8 | data[1]);
    return angle6 ;
}

float convertRawAngleToDegrees1(int16_t newAngle)
{
    /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
    float retVal = newAngle * 0.087890625;
    return retVal;
}