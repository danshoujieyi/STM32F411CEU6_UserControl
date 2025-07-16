// Created by ���ο� on 25-4-11.
//

#ifndef CTRBOARD_H7_ALL_KALMANFILTERONE_H
#define CTRBOARD_H7_ALL_KALMANFILTERONE_H

#include <stdint.h>

/* ������ -------------------------------------------------*/
#define NUM_JOINTS 6  // ����ͨ��������,��ʾ6��������

// ȡ��ע��������ʹ��˫����
// #define USE_DOUBLE_PRECISION

#ifdef USE_DOUBLE_PRECISION
typedef double mat_type_t;
#else
typedef float mat_type_t;
#endif
/* -------------------------------------------------------*/
#define KALMAN_F  1.0f     // ״̬ת�ƾ���һάʱΪ������
#define KALMAN_H  1.0f     // �۲����һάʱΪ������
#define KALMAN_Q  0.08f
#define KALMAN_R  0.3f

//Q����������������ӳԤ��ģ�͵Ĳ�ȷ���ԡ�
//Q����ϵͳ��ΪԤ��ģ�Ͳ��ɿ�������������ֵ����Ӧ�죬���������У���
//Q��С������Ԥ��ģ�ͣ��˲������ƽ������Ӧ���������Ժã���
//R���۲�����������ӳ�������Ĳ���������
//R������Ϊ�����������󣬸�����Ԥ��ֵ�����ƽ���������ͺ󣩡�
//R��С�����δ��������˲������������ֵ����Ӧ�죬�������󣩡�
//Q/R��ֵ������ Q/R ʹ��Ӧ���죬��С Q/R ʹ�����ƽ����

// ���������ʾ���ڵ�һ�ν��յ�����
// ����x0��ʼ״̬ÿ�ζ���0.0f��ʼ��ֱ�Ӵ��Զ����������һ�δ����������ݿ�ʼ�������ӿ������ٶȣ���Ҫ�����Զ��������ʵʱ�޸�
#define FIRST_MEASUREMENT_1 182.248f
#define FIRST_MEASUREMENT_2 312.626f
#define FIRST_MEASUREMENT_3 108.368f
#define FIRST_MEASUREMENT_4 143.085f
#define FIRST_MEASUREMENT_5 261.453f
#define FIRST_MEASUREMENT_6 124.232f

typedef struct {
    // ϵͳģ�;���
    mat_type_t F;  // ״̬ת�ƾ���
    mat_type_t H;  // �۲����

    // ����Э�������
    mat_type_t Q;  // ��������
    mat_type_t R;  // �۲�����

    // ״̬����
    mat_type_t x;  // ״̬����
    mat_type_t P;  // �������Э����

    // ����������
    mat_type_t K;
} KalmanFilter;

void Init_KalmanFiltersOne(mat_type_t F, mat_type_t H, mat_type_t Q, mat_type_t R);
void KalmanFilterOne_Init(KalmanFilter *kf,
                          mat_type_t F,
                          mat_type_t H,
                          mat_type_t Q,
                          mat_type_t R,
                          mat_type_t x0);

void KalmanFilterOne_Predict(KalmanFilter *kf);
mat_type_t KalmanFilterOne_Update(KalmanFilter *kf, mat_type_t z);
void KalmanFilterOne_Data(float *measurements, mat_type_t *filtered_data);

#endif //CTRBOARD_H7_ALL_KALMANFILTERONE_H



//
//// Created by ���ο� on 25-4-11.
////
//#include "arm_math.h"
//#include <stdint.h>
//
///* ������ -------------------------------------------------*/
//#define NUM_JOINTS          6       // �ؽ�����
//#define STATE_DIM           2       // ״̬ά�ȣ�λ��+�ٶȣ�
//#define MEASURE_DIM         1       // �۲�ά��
//#define PREDICT_TIME        0.3f    // ��ǰԤ��ʱ�䣨�룩
//
//// ȡ��ע��������ʹ��˫����
//// #define USE_DOUBLE_PRECISION
//#ifdef USE_DOUBLE_PRECISION
//typedef double mat_type_t;
//#define ARM_MAT_INIT  arm_mat_init_f64
//#define ARM_MAT_MULT  arm_mat_mult_f64
//#define ARM_MAT_TRANS arm_mat_trans_f64
//#define ARM_MAT_ADD   arm_mat_add_f64
//#define ARM_MAT_SUB   arm_mat_sub_f64
//#define ARM_MAT_SCALE arm_mat_scale_f64
//#define ARM_MAT_INV   arm_mat_inverse_f64
//#define ARM_MATRIX_INSTANCE arm_matrix_instance_f64
//#else
//typedef float mat_type_t;
//#define ARM_MAT_INIT  arm_mat_init_f32
//#define ARM_MAT_MULT  arm_mat_mult_f32
//#define ARM_MAT_TRANS arm_mat_trans_f32
//#define ARM_MAT_ADD   arm_mat_add_f32
//#define ARM_MAT_SUB   arm_mat_sub_f32
//#define ARM_MAT_SCALE arm_mat_scale_f32
//#define ARM_MAT_INV   arm_mat_inverse_f32
//#define ARM_MATRIX_INSTANCE arm_matrix_instance_f32
//#endif

