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
#include <stdio.h>
#include <stdbool.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"
#include "filter.h"
#include "arm_math.h"
#include "Hardware_i2c1.h"
#include "iic7.h"

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
static uint32_t algorithm_task_dwt = 0;   // ������
static float algorithm_task_dt = 0;       // �߳�ʵ������ʱ��dt
static float algorithm_task_delta = 0;    // ����߳�����ʱ��
static float algorithm_task_start_dt = 0; // ����߳̿�ʼʱ��
/* -------------------------------- ���Լ���߳���� --------------------------------- */




// ����AS5600��ȡ�ĽǶ�ֵ
static float angles_encoder_temp[7] = {0}; // ȫ�ֹ�������
static float angles_encoder_last[7] = {0.02f}; // ȫ�ֹ�������
static float angles_encoder[7] = {0}; // ȫ�ֹ�������


static int16_t raw_angle[7] = {0}; // ȫ�ֹ������
static int16_t raw_angle_last[7] = {0};
static AngleFilter filters[7]; // ���� 6 ����������Ӧ���˲���

static uint8_t flag = 0; // ��־λ��

extern QueueHandle_t xQueue;      // ���о��

#define ANGLE_ERROR_MIN 4.0f   // �Ƕ��ж�ֵ��С��4������Ϊ����Ч����
#define ANGLE_ERROR_MAX 358.0f // �Ƕ��ж�ֵ������358������Ϊ����Ч����


/* -------------------------------- �߳���� ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- �����ʼ������ ------------------------------- */
    // ��ʼ��ÿ���˲���
    for (int i = 0; i < 7; i++)
    {
        initAngleFilter(&filters[i]);
    }

    for(uint8_t i = 0; i < 20; i++){
        raw_angle_last[0] =getRawAngle2();
        raw_angle_last[1] =getRawAngle3();
        raw_angle_last[2] =getRawAngle4();
        raw_angle_last[3] =Read_Encoder_Angle4();
        raw_angle_last[4] =Read_Encoder_Angle5();
        raw_angle_last[5] =Read_Encoder_Angle6();
    //    raw_angle_last[6] =getRawAngle7();
    }
    flag = 1; // ��־λ��λ

/* -------------------------------- �����ʼ������ ------------------------------- */

/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- �̼߳�Topics��ʼ�� ------------------------------- */
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
    for(;;)
    {
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- ���Լ���̵߳��� --------------------------------- */
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- �̶߳���Topics��Ϣ ------------------------------- */

/* -------------------------------- �̴߳����д���� ------------------------------- */
        if(flag == 1) {
            // ģ���ȡ��������ԭʼ�Ƕ�ֵ��ʵ����Ӧ�滻Ϊ�������ӿڣ�
            if (raw_angle_last[0] > 0){
                raw_angle[0] = getRawAngle2();
                angles_encoder_temp[0] = processAngle(&filters[0], raw_angle[0]);// ����������Ƕȣ���ȡ�˲���ȥ����С���ֵ����ƽ��ֵ����ĽǶ�ֵ
                raw_angle_last[0] = raw_angle[0];
            }

            if (raw_angle_last[1] > 0) {
                raw_angle[1] = getRawAngle3();
                angles_encoder_temp[1] = processAngle(&filters[1], raw_angle[1]);
                raw_angle_last[1] = raw_angle[1];
            }

            if (raw_angle_last[2] > 0) {
                raw_angle[2] = getRawAngle4();
                angles_encoder_temp[2] = processAngle(&filters[2], raw_angle[2]);
                raw_angle_last[2] = raw_angle[2];
            }

            if (raw_angle_last[3] > 0) {
                raw_angle[3] = Read_Encoder_Angle4();
                angles_encoder_temp[3] = processAngle(&filters[3], raw_angle[3]);
                raw_angle_last[3] = raw_angle[3];
            }

            if (raw_angle_last[4] > 0) {
                raw_angle[4] = Read_Encoder_Angle5();
                angles_encoder_temp[4] = processAngle(&filters[4], raw_angle[4]);
                raw_angle_last[4] = raw_angle[4];
            }

            if (raw_angle_last[5] > 0) {
                raw_angle[5] = Read_Encoder_Angle6();
                angles_encoder_temp[5] = processAngle(&filters[5], raw_angle[5]);
                raw_angle_last[5] = raw_angle[5];
            }

//            if (raw_angle_last[6] > 0) {
//                raw_angle[6] = getRawAngle7();
//                angles_encoder_temp[6] = processAngle(&filters[6], raw_angle[6]);
//                raw_angle_last[6] = raw_angle[6];
//            }

            for (uint8_t i = 0; i < 6; i++)
            {
                // �ж�������Ч�Բ�����
                if (angles_encoder_temp[i] > 0.01f) {
                    angles_encoder[i] = angles_encoder_temp[i]; // ����Ϊ���һ����Чֵ
                } else {
//                    angles_encoder[0] = 180.0f;
//                    angles_encoder[1] = 180.0f;
//                    angles_encoder[2] = 180.0f;
//                    angles_encoder[3] = 180.0f;
//                    angles_encoder[4] = 180.0f;
//                    angles_encoder[5] = 180.0f;
//                    angles_encoder[6] = 180.0f;
                }
            }


            // �����������ݷ������
            xQueueSend(xQueue, angles_encoder, 0);
        }


//printf("AlgorithmTask_Entry: algorithm_task_dt = %f\n", algorithm_task_dt);
//printf("AlgorithmTask_Entry: algorithm_task_delta = %f\n", algorithm_task_delta);
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