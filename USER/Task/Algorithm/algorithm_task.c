/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
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

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
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
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */




// 定义AS5600读取的角度值
static float angles_encoder_temp[7] = {0}; // 全局共享数组
static float angles_encoder_last[7] = {0.02f}; // 全局共享数组
static float angles_encoder[7] = {0}; // 全局共享数组


static int16_t raw_angle[7] = {0}; // 全局共享变量
static int16_t raw_angle_last[7] = {0};
static AngleFilter filters[7]; // 定义 6 个编码器对应的滤波器

static uint8_t flag = 0; // 标志位，

extern QueueHandle_t xQueue;      // 队列句柄

#define ANGLE_ERROR_MIN 4.0f   // 角度判定值，小于4度则认为是无效数据
#define ANGLE_ERROR_MAX 358.0f // 角度判定值，大于358度则认为是无效数据


/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    // 初始化每个滤波器
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
    flag = 1; // 标志位置位

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        if(flag == 1) {
            // 模拟获取编码器的原始角度值（实际中应替换为传感器接口）
            if (raw_angle_last[0] > 0){
                raw_angle[0] = getRawAngle2();
                angles_encoder_temp[0] = processAngle(&filters[0], raw_angle[0]);// 处理编码器角度，获取滤波（去掉最小最大值，求平均值）后的角度值
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
                // 判断数据有效性并处理
                if (angles_encoder_temp[i] > 0.01f) {
                    angles_encoder[i] = angles_encoder_temp[i]; // 保存为最后一个有效值
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


            // 将编码器数据放入队列
            xQueueSend(xQueue, angles_encoder, 0);
        }


//printf("AlgorithmTask_Entry: algorithm_task_dt = %f\n", algorithm_task_dt);
//printf("AlgorithmTask_Entry: algorithm_task_delta = %f\n", algorithm_task_delta);
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(6);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
///**
// * @brief chassis 线程中所有发布者初始化
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis 线程中所有订阅者初始化
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis 线程中所有发布者推送更新话题
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis 线程中所有订阅者获取更新话题
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */