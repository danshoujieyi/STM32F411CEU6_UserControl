// Created by ���ο� on 25-4-11.
//
/* �������˲��㷨��ʽ˵����
   Ԥ��׶Σ�
   1. ״̬������ƣ�x??? = F? * x????
   2. Э����������ƣ�P?? = F? * P??? * F?? + Q?

   ���½׶Σ�
   3. �в���㣺y? = z? - H? * x???
   4. �в�Э���S? = H? * P?? * H?? + R?
   5. ���������棺K? = P?? * H?? * S???
   6. ״̬������ƣ�x?? = x??? + K? * y?
   7. Э������£�P? = (I - K? * H?) * P??
*/

#include "KalmanFilterOne.h"

// ��Ӧ�ó����������˲�������
static KalmanFilter joint_filters[NUM_JOINTS];
const mat_type_t first_measurements[NUM_JOINTS] = {
        FIRST_MEASUREMENT_1,
        FIRST_MEASUREMENT_2,
        FIRST_MEASUREMENT_3,
        FIRST_MEASUREMENT_4,
        FIRST_MEASUREMENT_5,
        FIRST_MEASUREMENT_6
};

// ��ʼ��ÿ��ͨ���������ò�ͬ������
void Init_KalmanFiltersOne(mat_type_t F, mat_type_t H, mat_type_t Q, mat_type_t R) {
    for (int i = 0; i < NUM_JOINTS; i++) {
        mat_type_t x0 = first_measurements[i];  // ʹ�õ�һ�β���ֵ��Ϊ��ʼ״̬
        KalmanFilterOne_Init(&joint_filters[i], F, H, Q, R, x0);
    }
}


void KalmanFilterOne_Init(KalmanFilter *kf,
                          mat_type_t F,
                          mat_type_t H,
                          mat_type_t Q,
                          mat_type_t R,
                          mat_type_t x0) {
    // ��ʼ��ϵͳģ�;���
    kf->F = F;
    kf->H = H;

    // ��ʼ������Э�������
    kf->Q = Q;
    kf->R = R;

    // ��ʼ��״̬����
    kf->x = x0;

    // ��ʼ�����Э�������
    kf->P = 1.0f;
}

void KalmanFilterOne_Predict(KalmanFilter *kf) {
    // Ԥ��״̬��x??? = F * x????
    kf->x = kf->F * kf->x;

    // Ԥ��Э���P?? = F * P??? * F? + Q
    kf->P = kf->F * kf->P * kf->F + kf->Q;
}

mat_type_t KalmanFilterOne_Update(KalmanFilter *kf, mat_type_t z) {
    // ����в� y = z - Hx
    mat_type_t y = z - kf->H * kf->x;

    // ����в�Э���� S = HPH? + R
    mat_type_t S = kf->H * kf->P * kf->H + kf->R;

    // ���㿨�������� K = PH?S??
    kf->K = kf->P * kf->H / S;

    // ����״̬���� x = x + Ky
    kf->x = kf->x + kf->K * y;

    // ����Э������� P = (I - KH)P
    kf->P = (1 - kf->K * kf->H) * kf->P;

    return kf->x;
}

// �������� float data[6]
void KalmanFilterOne_Data(float *measurements, mat_type_t *filtered_data) {
    for (int ch = 0; ch < NUM_JOINTS; ch++) {
        KalmanFilterOne_Predict(&joint_filters[ch]);
        filtered_data[ch] = KalmanFilterOne_Update(&joint_filters[ch], measurements[ch]);
    }
}





