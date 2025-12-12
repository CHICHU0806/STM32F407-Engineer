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

void ImuTask::run() {;
    BMI088_Init();
    ImuTempControl_Init();
    DWT_Delay_ms(1000);

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

        debug_P = roll;
        debug_I = pitch;
        debug_D = yaw;

        DWT_Delay_ms(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 1024, osPriorityRealtime);
    }
}