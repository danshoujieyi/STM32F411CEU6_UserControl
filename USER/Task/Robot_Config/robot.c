//
// Created by ���ο� on 25-4-21.
//

#include "robot.h"
#include "usart_task.h"
#include "drv_dwt.h"
#include "main.h"
#include "tim.h"

QueueHandle_t xQueue = NULL;
uint8_t buf[1]={0};
uint32_t count = 0;

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(float)*7

void robot_init(void)
{
    // �ر��ж�,��ֹ�ڳ�ʼ�������з����ж�
    // �벻Ҫ�ڳ�ʼ��������ʹ���жϺ���ʱ������
    // ������,��ֻ����ʹ�� dwt ������ʱ
//    __disable_irq();
    xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (xQueue == NULL) {
        Error_Handler();
    }

    dwt_init(); // ��ʼ�� DWT ��ʱ��

    HAL_TIM_Base_Start(&htim2);

    usart_rx_semaphore_init(); // �����ź�����ʼ��
    usart_tx_semaphore_init(); // �����ź�����ʼ��


    // ��ʼ�����,�����ж�
//    __enable_irq();
}