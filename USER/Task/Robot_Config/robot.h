//
// Created by 刘嘉俊 on 25-4-21.
//

#ifndef CTRBOARD_H7_ALL_ROBOT_H
#define CTRBOARD_H7_ALL_ROBOT_H

#define CPU_FREQUENCY 480     /* CPU主频(mHZ) */

#include "stm32f4xx_hal.h" // 使用的芯片
#include "cmsis_os.h" // 使用的 OS 头文件


#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#define user_free vPortFree
#else
#define user_malloc malloc
#define user_malloc free
#endif



void robot_init(void);

#endif //CTRBOARD_H7_ALL_ROBOT_H
