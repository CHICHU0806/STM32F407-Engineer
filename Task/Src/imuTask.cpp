//
// Created by 20852 on 2025/11/21.
//

#include "imuTask.h"
#include <cmath>
#include "BMI088.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "ist8310driver.h"
#include "ImuTempControl.h"
#include "MahonyAHRS.h"
#include "debug_vars.h"

float gyro[3], accel[3], temp, mag[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float roll, pitch, yaw;
float roll_out, pitch_out, yaw_out;

// 上电零点
static float yaw_zero = 0.0f;
// 动态零偏
static float yaw_bias = 0.0f;
// 累积角度
static float yaw_total = 0.0f;

// 参数
#define GYRO_ZERO_THRESHOLD 0.005236f   // rad/s，对应0.3 deg/s
#define BIAS_ALPHA 0.999f          // 零偏滑动平均系数
#define LOOP_DT 0.001f             // 循环周期 1ms

void ImuTask::run() {;
    BMI088_Init();
    ImuTempControl_Init();
    DWT_Delay_ms(1000);

    // 上电时记录初始 yaw 作为零点
    BMI088_Read(gyro, accel, &temp);
    MahonyAHRSupdateIMU(q,
                        gyro[0],
                        gyro[1],
                        gyro[2],
                        accel[0],
                        accel[1],
                        accel[2]);
    yaw_zero = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
    yaw_zero *= (180.0f / M_PI);
    yaw_total = yaw_zero;

    for (;;) {
        //真实数据读取
        BMI088_Read(gyro, accel, &temp);

        //控温
        ImuTempControl_Update(45, temp, 0.001f);

        //Mahony算法更新四元数
        MahonyAHRSupdateIMU(q,
                         gyro[0],
                         gyro[1],
                         gyro[2],
                         accel[0],
                         accel[1],
                         accel[2]);


        //根据四元数计算欧拉角
        roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
        pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
        yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));

        //角度制转换
        roll *= (180.0f / M_PI);
        pitch *= (180.0f / M_PI);
        yaw *= (180.0f / M_PI);
        //bsp_can1_sendimudata(roll,pitch,yaw);

        // 动态零偏补偿
        float gyro_z = gyro[2];
        if (fabsf(gyro_z) < GYRO_ZERO_THRESHOLD) {
            yaw_bias = BIAS_ALPHA * yaw_bias + (1.0f - BIAS_ALPHA) * gyro_z;
        }

        // 积分得到实际 yaw_total
        yaw_total += (gyro_z - yaw_bias) * LOOP_DT;
        float yaw_temp = yaw_total*(180.0f / M_PI);

        //直接滤除因为接收过程中丢失而引起的低频毛刺，但是我认为这个地方还有优化的可能
        if (roll != 0.0f) {
            roll_out = roll;
        }
        if (pitch != 0.0f) {
            pitch_out = pitch;
        }
        if (yaw_temp != 0.0f) {
            yaw_out = yaw_temp;
        }

        debug_roll = roll;
        debug_pitch = pitch;
        debug_yaw = yaw_out;

        DWT_Delay_ms(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 1024, osPriorityAboveNormal);
    }
}