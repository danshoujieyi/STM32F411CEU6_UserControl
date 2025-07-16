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
#include "usart_task.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include "drv_dwt.h"

#include <stdio.h>
#include <stdarg.h>

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
static uint32_t usart_task_dwt = 0;   // ������
static float usart_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float usart_task_delta = 0;    // ����߳�����ʱ��
static float usart_task_start_dt = 0; // ����߳̿�ʼʱ��
/* -------------------------------- ���Լ���߳���� --------------------------------- */

// �Զ������������
#define CUSTOMER_CONTROLLER_BUF_SIZE 256
static volatile uint8_t usart1_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart1_rx_size = 0;
static uint8_t usart1_rx_buffer[2][CUSTOMER_CONTROLLER_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART1_RX = NULL;           // ֪ͨ�������ź���

// ��˹ң����
#define SBUS_RX_BUF_SIZE 256
static volatile uint8_t usart5_rx_buffer_index = 0;  // ��ǰʹ�õĽ��ջ�����
static volatile uint16_t usart5_rx_size = 0;
static uint8_t usart5_rx_buffer[2][SBUS_RX_BUF_SIZE];
static SemaphoreHandle_t xSemaphoreUART5_RX = NULL;

static QueueSetHandle_t xUartQueueSet = NULL; // ������ն��м����,ͳһ�������ж��ź���

#define USART1_TX_DEBUG_BUFFER_SIZE 256 // printf���ͻ�������С
static char usart1_tx_debug_buffer[USART1_TX_DEBUG_BUFFER_SIZE];
static SemaphoreHandle_t xSemaphoreUART1_TX = NULL; // ����1�����ź���

extern struct referee_fdb_msg referee_fdb;

void usart_rx_semaphore_init(void)
{
    // FreeRTOS ��ʼ��
    xSemaphoreUART1_RX = xSemaphoreCreateBinary();  // <-- �ڴ˴������ź���
    xSemaphoreUART5_RX = xSemaphoreCreateBinary();  // <-- �ڴ˴������ź���
    // ������м��������� 3 ���ź�����
    xUartQueueSet = xQueueCreateSet(3);
    // ���������ź���������м�
    xQueueAddToSet(xSemaphoreUART1_RX, xUartQueueSet);
    xQueueAddToSet(xSemaphoreUART5_RX, xUartQueueSet);
}

void usart_tx_semaphore_init(void)
{
    // ������ֵ�ź���������DMA����ͬ����
    xSemaphoreUART1_TX = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphoreUART1_TX);
}

void USART1_RX_DMA_Init(void) {
    memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));
    //ʹ��DMA���ڽ���
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE);
    // �ر�DMA�Ĵ�������жϣ�����������ж�
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

//void USART5_RX_DMA_Init(void) {
//    memset(usart5_rx_buffer, 0, SBUS_RX_BUF_SIZE);
//    // �ر�DMA�Ĵ�������жϣ�����������ж�
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // ������Ϻ�����
//    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
//}

//void USART10_RX_DMA_Init(void) {
//    memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));
//    //ʹ��DMA���ڽ���
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE);
//    // �ر�DMA�Ĵ�������жϣ�����������ж�
//    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
//}

void process_usart1_rx_data(void) {
    uint8_t finishedBuffer;

    // ��������ȡ�ź�������Ϊ���м���ȷ���ź�����Ч��
    if (xSemaphoreTake(xSemaphoreUART1_RX, 0) == pdTRUE) {
        finishedBuffer = usart1_rx_buffer_index ^ 1;
        /* ���ݽ��� */
       // referee_data_unpack(usart1_rx_buffer[finishedBuffer], usart1_rx_size);

        // �������н��ջ���������
        memset(usart1_rx_buffer[finishedBuffer], 0, CUSTOMER_CONTROLLER_BUF_SIZE);
        // ֻ���㵱ǰʹ�õĻ���������
       // memset(usart1_rx_buffer[finishedBuffer], 0, usart1_rx_size);

    }
}

//void process_uart5_rx_data(void) {
//    uint8_t finishedBuffer;
//
//    if (xSemaphoreTake(xSemaphoreUART5_RX, 0) == pdTRUE) {
//        finishedBuffer = usart5_rx_buffer_index ^ 1;
//        /* SBUSЭ����� */
//        sbus_data_unpack(usart5_rx_buffer[finishedBuffer], usart5_rx_size);
//
//        memset(usart5_rx_buffer[finishedBuffer], 0, SBUS_RX_BUF_SIZE);
//    }
//}

//void process_uart10_rx_data(void) {
//    uint8_t finishedBuffer;
//
//    if (xSemaphoreTake(xSemaphoreUART10_RX, 0) == pdTRUE) {
//        finishedBuffer = referee_rx_buffer_index ^ 1;
//        /* ����ϵͳ���ݽ��� */
//        referee_data_unpack(referee_rx_buffer[finishedBuffer], referee_rx_size);
//
//        memset(referee_rx_buffer[finishedBuffer], 0, REFEREE_RX_BUF_SIZE);
//    }
//}


/* -------------------------------- �߳���� ------------------------------- */
void UsartTask_Entry(void const * argument)
{
/* -------------------------------- �����ʼ������ ------------------------------- */
    QueueSetMemberHandle_t xActivatedMember;

  //  USART1_RX_DMA_Init();
    //USART5_RX_DMA_Init();
    //USART10_RX_DMA_Init();
/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    usart_task_dt = dwt_get_delta(&usart_task_dwt);
    usart_task_start_dt = dwt_get_time_ms();
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    for(;;)
    {
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
        usart_task_delta = dwt_get_time_ms() - usart_task_start_dt;
        usart_task_start_dt = dwt_get_time_ms();
        usart_task_dt = dwt_get_delta(&usart_task_dwt);
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */

/* -------------------------------- �̴߳����д���� ------------------------------- */
        xActivatedMember = xQueueSelectFromSet(xUartQueueSet, portMAX_DELAY);
//
//        // �жϴ���Դ���������ݣ����ݴ������ݵ���Ҫ�̶ȵ�������˳��
//        if (xActivatedMember == xSemaphoreUART1_RX) {
//            process_usart1_rx_data();  // �Զ��������
//        } else if (xActivatedMember == xSemaphoreUART5_RX) {
//            // process_uart5_rx_data();  // ��˹ң����
//        }
////        } else if (xActivatedMember == xSemaphoreUART10_RX) {
////            process_uart10_rx_data(); // ����ϵͳ����ܣ�
////        }
/* -------------------------------- �̴߳����д���� ------------------------------- */

/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- �̷߳���Topics��Ϣ ------------------------------- */
        vTaskDelay(5);
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
    if (huart->Instance == USART1)  // �޸��ж�����
    {
        // �жϽ��յ����ݴ�С�Ƿ����ƣ�����������򲻴���
        if (Size > CUSTOMER_CONTROLLER_BUF_SIZE)
        {
            return;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        usart1_rx_size = Size;
        usart1_rx_buffer_index = usart1_rx_buffer_index ^ 1;

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1_rx_buffer[usart1_rx_buffer_index],CUSTOMER_CONTROLLER_BUF_SIZE);

        xSemaphoreGiveFromISR(xSemaphoreUART1_RX, NULL);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

extern uint8_t dma_busy;
extern uint8_t dma_busy2;
// DMA������ɻص�����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        dma_busy2 = 0; // ��־λ��Ϊ����
    }

    if (huart->Instance == USART6) {
        dma_busy = 0; // ��־λ��Ϊ����
    }
    // ����Ƿ���USART1��DMA�������
    if (huart->Instance == USART1) {
        // �ͷ��ź�������ʾDMA���Դ�����һ��������
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xSemaphoreUART1_TX, &xHigherPriorityTaskWoken);
        // �����Ҫ���Ѹ������ȼ������򴥷��������л�
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


//void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
//{
//    if(huart->Instance == UART5)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_rx_buffer[usart5_rx_buffer_index], SBUS_RX_BUF_SIZE); // ���շ������������
//        memset(usart5_rx_buffer, 0, sizeof(usart5_rx_buffer));							   // ������ջ���
//    }
//
//    if(huart->Instance == USART10)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buffer[referee_rx_buffer_index], REFEREE_RX_BUF_SIZE); // ���շ������������
//        memset(referee_rx_buffer, 0, sizeof(referee_rx_buffer));// ���˫����
//    }
//
//    if(huart->Instance == USART1)
//    {
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buffer[usart1_rx_buffer_index], CUSTOMER_CONTROLLER_BUF_SIZE); // ���շ������������
//        memset(usart1_rx_buffer, 0, sizeof(usart1_rx_buffer));// ���˫����
//    }
//}

// �Զ���DebugPrintf����������printf��
//TODO:ʵ��˫�������л���Ч����������һ��buff[0]���ͣ���һ��buff[1]���գ�Ŀǰ��buff[0]���������̷��ͣ�ͬʱ�л���buff[1]���������̷���
void USART1_DebugPrintf(const char *format, ...) {
    va_list args;
    uint16_t length;

    // �ȴ�DMA������ɣ���ȡ�ź�����
    if (xSemaphoreTake(xSemaphoreUART1_TX, portMAX_DELAY) == pdTRUE) {

        // ʹ�ÿɱ������ʽ���ַ�������ǰ������
        va_start(args, format);
        length = vsnprintf(usart1_tx_debug_buffer, USART1_TX_DEBUG_BUFFER_SIZE, format, args);
        va_end(args);

        // ȷ���ַ���������Ч
        if (length > 0) {
            // ʹ��DMA�첽���͵�ǰ����������
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)usart1_tx_debug_buffer, length);
        } else {
            // ������Чʱֱ���ͷ��ź���
            xSemaphoreGive(xSemaphoreUART1_TX);
        }
    }
}