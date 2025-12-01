//
// Created by 20852 on 2025/11/21.
//

#include "imuTask.h"
#include "BMI088.h"
#include "bsp_dwt.h"
#include "ist8310driver.h"
#include "ImuTempControl.h"
#include "MahonyAHRS.h"

float gyro[3], accel[3], temp, mag[3];
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

void ImuTask::run() {;
    BMI088_Init();
    ist8310_init();
    ImuTempControl_Init();

    for (;;) {
        BMI088_Read(gyro, accel, &temp);
        ist8310_read_mag(mag);
        DWT_Delay_ms(1);
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
        DWT_Delay_ms(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 1024, osPriorityRealtime);
    }
}