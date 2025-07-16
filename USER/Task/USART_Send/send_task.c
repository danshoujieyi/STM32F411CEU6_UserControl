/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   �������㷨�����̣߳��������㷨�������������߳��м����������
  ******************************************************************************
  * @attention
  *
  * ��������ѭGPLv3��ԴЭ�飬����ѧϰ����ʹ��
  * δ����ɲ���������ҵ��;
  *
  ******************************************************************************
  */

#include "send_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include <string.h>
#include <stdio.h>
#include "crc8_crc16.h"
#include "usart.h"
#include "usart_task.h"
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
//static struct chassis_cmd_msg chassis_cmd;
//static struct chassis_fdb_msg chassis_fdb;
//static struct trans_fdb_msg trans_fdb;
//static struct ins_msg ins_data;
//
//static publisher_t *pub_chassis;
//static subscriber_t *sub_cmd,*sub_ins,*sub_trans;
//
//static void chassis_pub_init(void);
//static void chassis_sub_init(void);
//static void chassis_pub_push(void);
//static void chassis_sub_pull(void);
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
/* -------------------------------- ���Լ���߳���� --------------------------------- */
static uint32_t send_task_dwt = 0;   // ������
static float send_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float send_task_delta = 0;    // ����߳�����ʱ��
static float send_task_start_dt = 0; // ����߳̿�ʼʱ��
/* -------------------------------- ���Լ���߳���� --------------------------------- */

static uint8_t dma_tx_buffer[2][FRAME_SIZE]; // ˫������
static volatile uint8_t current_buffer = 0;           // ��ǰ����������
uint8_t dma_busy = 0;        // DMA״̬��־λ
uint8_t dma_busy2 = 0; // DMA״̬��־λ2����ѡ���Ӿ������������

static float angles[7] = {0.0f}; // ������ʱ�洢�����ж�ȡ�ı�����ֵ
static float encoder_values[7] = {0, 0, 0, 0, 0, 0,0};              // �洢6��������ֵ

extern QueueHandle_t xQueue;    // FreeRTOS ���о��
void PackData(float *values, uint16_t data_length, RobotArmController_t *tx_data)
{
    static uint32_t frame_seq = 0; // ֡���
    // ����֡ͷ
    tx_data->frame_header.sof = 0xA5;                      // ��ʼ�ֽ�
    tx_data->frame_header.data_length = data_length;        // ���ݶγ���
    tx_data->frame_header.seq = frame_seq++;                // �����
    if (frame_seq == 0) frame_seq = 1;  // ��ֹ�������������֧��255�������

    // ����֡ͷCRC8
    tx_data->frame_header.crc8 = 0;  // ��ʼCRC8
    append_CRC8_check_sum((uint8_t *)(&tx_data->frame_header), FRAME_HEADER_LENGTH);  // ���CRC8У��

    // ����������
    tx_data->cmd_id = ARM_CONTROLLER_CMD_ID;

    // ��������6��float��ÿ��������ֵ4�ֽڣ�
    // �������ݶ�
    for (int i = 0; i < 7; i++) {
        uint8_t *src = (uint8_t *)&values[i];
        uint8_t *dst = &tx_data->data[i * 4];
        memcpy(dst, src, sizeof(float)); // �Զ�����4���ֽ�
    }


    // ����֡βCRC16
    tx_data->frame_tail = 0;  // ��ʼCRC16
    append_CRC16_check_sum((uint8_t *)tx_data, FRAME_SIZE );  // ���CRC16У��
}

void DMA_Send_Frame(void)
{
    if (dma_busy == 0) { // �ж�DMA�Ƿ����
        dma_busy = 1;    // ��־λ��Ϊæ
        // ����DMA����
        HAL_UART_Transmit_DMA(&huart6, dma_tx_buffer[current_buffer] , FRAME_SIZE);
        // �л�����һ��������
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}

void DMA_Send_Frame2(void)
{
    if (dma_busy2 == 0) { // �ж�DMA�Ƿ����
        dma_busy2 = 1;    // ��־λ��Ϊæ
        // ����DMA����
        HAL_UART_Transmit_DMA(&huart2, dma_tx_buffer[current_buffer] , FRAME_SIZE);
        // �л�����һ��������
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}

// DMA������ɻص�����
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART6) {
//        dma_busy = 0; // ��־λ��Ϊ����
//    }
//}

/* -------------------------------- �߳���� ------------------------------- */
void SendTask_Entry(void const * argument)
{
/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    send_task_dt = dwt_get_delta(&send_task_dwt);
    send_task_start_dt = dwt_get_time_ms();
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    for(;;)
    {
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
        send_task_delta = dwt_get_time_ms() - send_task_start_dt;
        send_task_start_dt = dwt_get_time_ms();
        send_task_dt = dwt_get_delta(&send_task_dwt);
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */

/* -------------------------------- �̴߳����д���� ------------------------------- */

        //�Ӷ����л�ȡ������ֵ
        if (xQueueReceive(xQueue, angles, 0) == pdTRUE) {
            // ����ȫ�ֵ� encoder_values ���飨��ѡ��
            for (int i = 0; i < 6; i++) {
                encoder_values[i] = angles[i];
            }

            //������ݵ�tx_data�ṹ��
            RobotArmController_t tx_data = {0};  // �������ݰ��ṹ��
            PackData(encoder_values, 30, &tx_data);  // �������֡
            // ������������д��DMA������
            memcpy(dma_tx_buffer[current_buffer], &tx_data, FRAME_SIZE);
            // ���� DMA ����
            //DMA_Send_Frame2(); // ��������֡
            DMA_Send_Frame();
           // printf("Send Data\n");
        }

/* -------------------------------- �̴߳����д���� ------------------------------- */

/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
        vTaskDelay(6);
    }
}
/* -------------------------------- �߳̽��� ------------------------------- */

/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */
///**
// * @brief chassis �߳������з����߳�ʼ��
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis �߳������ж����߳�ʼ��
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis �߳������з��������͸��»���
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis �߳������ж����߻�ȡ���»���
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- �̼߳�ͨѶTopics��� ------------------------------- */