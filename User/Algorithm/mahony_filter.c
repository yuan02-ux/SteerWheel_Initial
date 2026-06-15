
#include "mahony_filter.h"

#include "arm_math.h"//无

struct MAHONY_FILTER_t mahony_filter;

/**
 * @brief �Ż�����ƽ��������֧��FPU��ARM�ϣ�Ӳ�������Ϳ�������
 */
static float32_t arm_invSqrt(float32_t x) {
    float32_t out;
    arm_status status = arm_sqrt_f32(x, &out);
    if (status != ARM_MATH_SUCCESS || out == 0.0f) return 0.0f;
    return 1.0f / out;
}

/**
 * @brief ������Ԫ��������ת����
 * @param f ָ��MAHONY_FILTER_t�ṹ���ָ�룬������Ԫ������ת����洢����
 * @return void
 */
void RotationMatrix_update(struct MAHONY_FILTER_t *f)
{
    float32_t q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;

    // ��ǰ����ƽ��������ظ��˷�
    float32_t q1q1 = q1 * q1;
    float32_t q2q2 = q2 * q2;
    float32_t q3q3 = q3 * q3;
    float32_t q0q1 = q0 * q1;
    float32_t q0q2 = q0 * q2;
    float32_t q0q3 = q0 * q3;
    float32_t q1q2 = q1 * q2;
    float32_t q1q3 = q1 * q3;
    float32_t q2q3 = q2 * q3;

    f->rMat[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    f->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    f->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    f->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    f->rMat[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    f->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    f->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    f->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    f->rMat[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

/**
 * @brief Mahony�˲��㷨���ĸ��º��� (�����ٶȼƵ�ͨ�˲���)
 * @param f ָ��MAHONY_FILTER_t�ṹ���ָ��
 * @param gx, gy, gz ������ԭʼ���� (rad/s)
 * @param ax, ay, az ���ٶȼ�ԭʼ���� (m/s2)
 * @param dt �������� (s)
 */
void mahony_update(struct MAHONY_FILTER_t *f,
                   float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
{
    f->dt = dt;
    float halfT = 0.5f * f->dt;

    // ���ٶȼ�һ�׵�ͨ�˲�
    f->acc_lpf.x = (1.0f - f->alpha) * f->acc_lpf.x + f->alpha * ax;
    f->acc_lpf.y = (1.0f - f->alpha) * f->acc_lpf.y + f->alpha * ay;
    f->acc_lpf.z = (1.0f - f->alpha) * f->acc_lpf.z + f->alpha * az;

    // �����˲���ļ��ٶ�ģ��
    float32_t acc_sum_sq = f->acc_lpf.x * f->acc_lpf.x +
                           f->acc_lpf.y * f->acc_lpf.y +
                           f->acc_lpf.z * f->acc_lpf.z;
    arm_sqrt_f32(acc_sum_sq, &f->acc_norm);

    // ����������ģ��
    float32_t gyro_sum_sq = gx * gx + gy * gy + gz * gz;
    float32_t gyro_norm;
    arm_sqrt_f32(gyro_sum_sq, &gyro_norm);

    // ��̬��ƫѧϰ�߼�
    int is_static = (fabsf(f->acc_norm - 9.81f) < 0.1f) && (gyro_norm < 0.015f);
    if (is_static)
    {
        const float learn_rate = 0.006f;
        f->gyro_bias.x = (1 - learn_rate) * f->gyro_bias.x + learn_rate * gx;
        f->gyro_bias.y = (1 - learn_rate) * f->gyro_bias.y + learn_rate * gy;
        f->gyro_bias.z = (1 - learn_rate) * f->gyro_bias.z + learn_rate * gz;
    }

    // ȥ����������ƫ
    gx -= f->gyro_bias.x;
    gy -= f->gyro_bias.y;
    gz -= f->gyro_bias.z;

    // ���ٶȼ���Ч���ж�
    int high_dynamic = (fabsf(f->acc_norm - 9.81f) > 1.5f) || (gyro_norm > 1.0f);

    if (!high_dynamic && acc_sum_sq > 0.000001f)
    {
        // ��һ���˲���ļ��ٶȼ�����
        float32_t norm = arm_invSqrt(acc_sum_sq);
        float ax_n = f->acc_lpf.x * norm;
        float ay_n = f->acc_lpf.y * norm;
        float az_n = f->acc_lpf.z * norm;

        // ��˼���������� (�����ڻ���ϵͶӰ f->rMat[2] �� ���ٶȼƲ������� �����)
        // ex = ay_meas * rMat[2][2] - az_meas * rMat[2][1]
        // ey = az_meas * rMat[2][0] - ax_meas * rMat[2][2]
        float ex = ay_n * f->rMat[2][2] - az_n * f->rMat[2][1];
        float ey = az_n * f->rMat[2][0] - ax_n * f->rMat[2][2];

        // �������ۼ�
        if (gyro_norm < 0.5f)
        {
            f->exInt += f->Ki * ex * dt;
            f->eyInt += f->Ki * ey * dt;
        }
        else
        {
            // ��̬�ϴ�ʱ����ѡ���û��ֻ���˥�������߱��ֲ���
            f->exInt *= 0.99f;
            f->eyInt *= 0.99f;
        }

        // ע�벹������ٶ�
        gx += f->Kp * ex + f->exInt;
        gy += f->Kp * ey + f->eyInt;
    }

    // ��Ԫ�����ָ��� (Runge-Kutta 1��)
    float q0 = f->q0, q1 = f->q1, q2 = f->q2, q3 = f->q3;
    f->q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    f->q1 += ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    f->q2 += ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    f->q3 += ( q0 * gz + q1 * gy - q2 * gx) * halfT;

    // ��Ԫ����һ��
    float q_norm = arm_invSqrt(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= q_norm; f->q1 *= q_norm; f->q2 *= q_norm; f->q3 *= q_norm;

    // ������ת������һ���ڼ��㼰���ʹ��
    RotationMatrix_update(f);
}

/**
 * @brief ����ת������㲢�����̬�ǣ�����/���/ƫ����
 * @param f ָ��MAHONY_FILTER_t�ṹ���ָ�룬�洢��ת�������̬�ǽ��
 * @return void
 */


//void mahony_output(struct MAHONY_FILTER_t *f) {
//    f->pitch = -asinf(f->rMat[2][0]) * (180.0f / PI);
//    f->roll  = atan2f(f->rMat[2][1], f->rMat[2][2]) * (180.0f / PI);
//    f->yaw   = atan2f(f->rMat[1][0], f->rMat[0][0]) * (180.0f / PI);
//    // �ۻ�ƫ���Ǽ��㣺��ƫ���Ǳ仯����180��ʱ����Ϊ���������䣬�����ۻ�����
//    float yaw_diff = f->yaw - f->last_yaw;
//    if (yaw_diff > 180.0f) {
//        yaw_diff -= 360.0f;
//    } else if (yaw_diff < -180.0f) {
//        yaw_diff += 360.0f;
//    }
//    // �ۼӵ���ƫ����
//    f->YawTotalAngle += yaw_diff;
//    // ������һ�ε�ƫ���ǣ�������һ�μ���
//    f->last_yaw = f->yaw;
//}

void mahony_output(struct MAHONY_FILTER_t *f) {
    float r20 = f->rMat[2][0];
    if (r20 > 1.0f) r20 = 1.0f;
    if (r20 < -1.0f) r20 = -1.0f;

    float sqrt_val;
    arm_sqrt_f32(1.0f - r20 * r20, &sqrt_val);
    
    // 1. ���㵱ǰ֡δ�˲���ԭʼŷ���� (Raw Angles)
    float raw_pitch = -atan2f(r20, sqrt_val) * (180.0f / PI);
    float raw_roll  = atan2f(f->rMat[2][1], f->rMat[2][2]) * (180.0f / PI);
    float raw_yaw   = atan2f(f->rMat[1][0], f->rMat[0][0]) * (180.0f / PI);

    /**
     * 2. ���������һ�׵�ͨ�˲�ϵ�� (out_alpha)
     * out_alpha ԽС������Խƽ������Ƶ�����˳�Խ���ף��������������΢С����λ�ͺ�
     * ���� < 0.01 �ȵ�΢С��Ƶ������ȡֵ�� 0.15f ~ 0.30f ֮����Դﵽ���� EKF ������ƽ��Ч����
     */
    const float out_alpha = 0.20f; 

    // 3. �� Pitch �� Roll ����һ�׵�ͨ�˲�
    f->pitch = (1.0f - out_alpha) * f->pitch + out_alpha * raw_pitch;
    f->roll  = (1.0f - out_alpha) * f->roll + out_alpha * raw_roll;

    // 4. �� Yaw ����һ�׵�ͨ�˲������봦�� -180 �� 180 �ȵ�����ֽ��ߣ�
    float yaw_error = raw_yaw - f->yaw;
    if (yaw_error > 180.0f)       yaw_error -= 360.0f;
    else if (yaw_error < -180.0f) yaw_error += 360.0f;
    
    f->yaw += out_alpha * yaw_error;
    
    // �淶���˲���� f->yaw �� [-180, 180] ��Χ��
    if (f->yaw > 180.0f)        f->yaw -= 360.0f;
    else if (f->yaw < -180.0f)  f->yaw += 360.0f;

    // 5. �����˲���ĺ���ǵ����������������ۼ�ƫ���� YawTotalAngle
    // ��ʱ f->last_yaw �ڲ����������һ֡�˲����ֵ
    float yaw_diff = f->yaw - f->last_yaw;
    if (yaw_diff > 180.0f)       yaw_diff -= 360.0f;
    else if (yaw_diff < -180.0f) yaw_diff += 360.0f;

    f->YawTotalAngle += yaw_diff;
    f->last_yaw = f->yaw; // ���� last_yaw Ϊ��ǰ֡�˲����ֵ������һ֡ʹ��
}
/**
 * @brief Mahony�˲��㷨��ʼ������
 * @param f ָ��MAHONY_FILTER_t�ṹ���ָ�룬����ʼ�����㷨�ṹ��
 * @param Kp �������棨��̬У��������
 * @param Ki �������棨��̬У��������
 * @param dt �㷨�������ڣ���λ���룩
 * @return void
 */
void mahony_init(struct MAHONY_FILTER_t *f, float Kp, float Ki, float alpha,float dt)
{
    f->Kp = Kp;
    f->Ki = Ki;
    f->alpha = alpha;
    f->dt = dt;

    f->q0 = 1; f->q1 = 0; f->q2 = 0; f->q3 = 0;

    f->acc_lpf.x = 0; f->acc_lpf.y = 0; f->acc_lpf.z = 0;

    f->gyro_bias.x = 0;
    f->gyro_bias.y = 0;
    f->gyro_bias.z = 0;

    f->pitch = 0;
    f->roll = 0;
    f->yaw = 0;
    f->YawTotalAngle = 0;

    f->exInt = f->eyInt = 0;

    RotationMatrix_update(f);
    osDelay(1);
}