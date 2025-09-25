//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "CascadePID.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;

int output = 0;

void MotorTask::run() {
    static int32_t target_angle = 0;
    const int32_t step = 2048; // 90度对应编码器量
    uint32_t last_time = osKernelSysTick(); // 记录上次更新时间

    angle_pid_clear(); // 清零积分

    for (;;) {
        uint32_t now = osKernelSysTick();
        //1s一次更新目标角度
        if (now - last_time >= 1000) {
            target_angle += step;
            if (target_angle >= 8192) {
                target_angle -= 8192; // 跨零处理
                }
            last_time = now;
        }

        int16_t output = angle_pid_calculate(target_angle, motor_1.rotor_angle,0.01f);
        bsp_can_sendmotorcmd(output, output, output, output);

        osDelay(5);
    }
}

extern "C" {
    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
