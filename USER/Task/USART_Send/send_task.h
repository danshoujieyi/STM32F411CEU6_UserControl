//
// Created by ���ο� on 25-6-10.
//

#ifndef CTRBOARD_H7_ALL_DJMOTOR_TASK_H
#define CTRBOARD_H7_ALL_DJMOTOR_TASK_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

// ��������֡�����ֳ���
#define FRAME_HEADER_LENGTH 5 // ֡ͷ����
#define CMD_ID_LENGTH 2       // �����볤��
#define DATA_LENGTH 30        // ���ݶγ���
#define FRAME_TAIL_LENGTH 2   // ֡β����
// ����֡���� (֡ͷ + ������ + ������ + ֡β)
// 5�ֽ�֡ͷ + 2�ֽ������� + 30�ֽ����� + 2�ֽ�֡β #define FRAME_SIZE 39
#define FRAME_SIZE (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // ��������֡����

#define ARM_CONTROLLER_CMD_ID 0x0302 // �����е�ۿ�����������

// �����е�ۿ��������ݰ��ṹ��
// ʹ�� `__attribute__((packed))` ���ýṹ���Ա�Ķ��룬ȷ���ṹ����յ��������ڴ��С�
// �������Ա�����Ϊ�ṹ���������¶��������ֽڣ�����֡ͷ�����ݶ�֮��Ķ����ֽڣ���
// �������ȷ���ṹ���еĳ�Ա�������У������ڴ��˷ѣ���������Ҫ��ȷ�������ݸ�ʽʱ�ǳ���Ҫ��

typedef struct __attribute__((packed)) { // ���ýṹ�����
    struct __attribute__((packed)){  // ����֡ͷ���ֽṹ��Ķ���
        uint8_t sof;              // ��ʼ�ֽڣ��̶�ֵΪ0xA5����ʾ����֡�Ŀ�ʼ
        uint16_t data_length;     // ���ݶγ��ȣ�2�ֽڣ�����ʾ���������ֽ���
        uint8_t seq;              // ����ţ�1�ֽڣ�����ʾ��ǰ���ݰ������к�
        uint8_t crc8;             // ֡ͷ CRC8 У�飨1�ֽڣ�����������У��
    } frame_header;               // ֡ͷ����

    uint16_t cmd_id;              // �����루2�ֽڣ�����ʾ��ǰ֡����������
    uint8_t data[30];             // ���ݶΣ�30�ֽڣ������ڴ��ʵ�ʵ�����
    uint16_t frame_tail;         // ֡β CRC16 У�飨2�ֽڣ�������ȷ�����ݴ���������
} RobotArmController_t;


// ����ƴ�Ӻ���
void Data_Concatenation(RobotArmController_t *tx_data, uint8_t *data, uint16_t data_length);


#endif //CTRBOARD_H7_ALL_DJMOTOR_TASK_H
