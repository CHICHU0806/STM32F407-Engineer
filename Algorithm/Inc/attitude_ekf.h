//
// Created by 20852 on 2025/11/21.
//

#ifndef STARTM3508_ATTITUDE_EKF_H
#define STARTM3508_ATTITUDE_EKF_H

#include <stdint.h>

struct EKF_Output {
    float q0,q1,q2,q3; // quaternion (w,x,y,z)
    float roll;  // radians
    float pitch; // radians
    float yaw;   // radians
};

class AttitudeEKF {
public:
    AttitudeEKF();

    // 初始化（可选给初始四元数，以世界系向上 [0,0,1]）
    void init();

    // main update: gyro (rad/s), accel (m/s^2), mag (uT or arbitrary), dt seconds
    // accel and mag can be unnormalized; function will normalize them.
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz,
                float dt);

    // 获取结果
    EKF_Output getOutput() const;

    // 可调参数（public for easy tuning）
    float gyro_noise;     // gyro noise variance (rad^2 / s)
    float gyro_bias_noise; // gyro bias random walk variance (rad^2 / s^3)
    float accel_noise;    // accelerometer measurement noise variance
    float mag_noise;      // magnetometer measurement noise variance

private:
    // state
    float q[4];   // quaternion w,x,y,z
    float bg[3];  // gyro bias

    // error-covariance P (6x6) for [delta_theta (3); delta_bg (3)]
    float P[6][6];

    // reference magnetic field in navigation frame (set on first valid mag read)
    float m_ref[3];
    bool m_ref_set;

    // internal helpers
    static void quatNormalize(float q[4]);
    static void quatMultiply(const float a[4], const float b[4], float out[4]);
    static void quatIntegrate(float q[4], const float w[3], float dt); // w in rad/s
    static void quatToDCM(const float q[4], float C[3][3]);
    static void dcmToEuler(const float C[3][3], float &roll, float &pitch, float &yaw);

    // small-matrix helpers
    static void mat3_mul_vec(const float M[3][3], const float v[3], float out[3]);
    static void cross_product(const float a[3], const float b[3], float out[3]);
    static void skew3(const float v[3], float M[3][3]);
    static void mat3_add(const float A[3][3], const float B[3][3], float out[3][3]);
    static void mat3_scale(const float A[3][3], float s, float out[3][3]);
    static void mat3_copy(const float A[3][3], float out[3][3]);
    static void mat3_inverse(const float A[3][3], float invA[3][3]); // assumes invertible
};

#endif //STARTM3508_ATTITUDE_EKF_H