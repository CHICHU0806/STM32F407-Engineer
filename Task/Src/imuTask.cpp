//
// Created by 20852 on 2025/11/21.
//

#include "imuTask.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "attitude_ekf.h"

AttitudeEKF attitude_filter;

float gyro[3], accel[3], temp, mag[3];

float roll ,pitch, yaw;

void ImuTask::run() {;
    BMI088_init();
    ist8310_init();

    attitude_filter.init();

    float last_time = DWT_GetSeconds();

    for (;;) {
        //--- 1. 读取9轴数据 ---
        BMI088_read(gyro, accel,&temp);
        ist8310_read_mag(mag);

        // --- 2. 计算 dt 时间 ---
        float now = DWT_GetSeconds();
        float dt = now - last_time;
        last_time = now;

        // --- 3. 调用 EKF 更新姿态 ---
        attitude_filter.update(gyro[0], gyro[1], gyro[2],
                               accel[0], accel[1],accel[2],
                               mag[0], mag[1], mag[2],
                                   dt);

        // --- 4. 获取姿态角 或 四元数 ---
        EKF_Output out = attitude_filter.getOutput();

        roll = out.roll ;  // 转为度
        pitch = out.pitch ;
        yaw = out.yaw ;

        // (然后你做其他任务，比如发布给底盘)
        DWT_Delay_ms(1); // 1ms
    }
}

extern "C" {
    static ImuTask imu_task;

    void ImuTask_Init() {
        imu_task.start((char*)"ImuTask", 512, osPriorityHigh);
    }
}