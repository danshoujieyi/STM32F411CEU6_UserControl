//
// Created by ���ο� on 25-4-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_H
#define CTRBOARD_H7_ALL_ROBOT_H

#define CPU_FREQUENCY 480     /* CPU��Ƶ(mHZ) */

#include "stm32f4xx_hal.h" // ʹ�õ�оƬ
#include "cmsis_os.h" // ʹ�õ� OS ͷ�ļ�


#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif



void robot_init(void);

#endif //CTRBOARD_H7_ALL_ROBOT_H
