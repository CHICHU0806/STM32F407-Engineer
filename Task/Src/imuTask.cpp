//
// Created by 20852 on 2025/11/21.
//

#include "imuTask.h"
#include <cmath>
#include "BMI088.h"
#include "bsp_dwt.h"
#include "ist8310driver.h"
#include "ImuTempControl.h"
#include "MahonyAHRS.h"

#define PI 3.14159265358979323846f

float gyro[3], accel[3], temp, mag[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float roll, pitch, yaw;

void ImuTask::run() {;
    BMI088_Init();
    ist8310_init();
    ImuTempControl_Init();

    for (;;) {
        BMI088_Read(gyro, accel, &temp);
        ist8310_read_mag(mag);

        ImuTempControl_Update(40, temp, 0.001f);

        MahonyAHRSupdate(q,
                         gyro[0],
                         gyro[1],
                         gyro[2],
                         accel[0],
                         accel[1],
                         accel[2],
                         mag[0],
                         mag[1],
                         mag[2]);

        roll  = atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
        pitch = asinf(2.0f*(q[0]*q[2] - q[3]*q[1]));
        yaw   = atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));

        roll *= (180.0f / PI);
        pitch *= (180.0f / PI);
        yaw *= (180.0f / PI);

        DWT_Delay_ms(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 1024, osPriorityRealtime);
    }
}