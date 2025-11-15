//
// Created by 20852 on 2025/10/5.
//
#include "omniwheelTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "omniwheel.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;

void OmniWheelTask::run() {
    for (;;) {
        // === 1. 从遥控器获取输入 ===
        float Vx =  dbus.ch[3] * 0.5f;   // 前后方向
        float Vy = -dbus.ch[2] * 0.5f;   // 左右方向
        float omega = -dbus.ch[0] * 0.5f; // 旋转速度

        // === 2. 调用全向轮解算 ===
        OmniWheel omni(0.5f, 0.5f, 0.075f);
        auto wheel_speeds = omni.OmniWheel_Calculate(Vx, Vy, omega);

        // === 3. PID计算输出 ===
        int16_t motor1_cmd = speed_pid_calculate(wheel_speeds[0], motor_1.rotor_speed, 0.01f);
        int16_t motor2_cmd = speed_pid_calculate(wheel_speeds[1], motor_2.rotor_speed, 0.01f);
        int16_t motor3_cmd = speed_pid_calculate(wheel_speeds[2], motor_3.rotor_speed, 0.01f);
        int16_t motor4_cmd = speed_pid_calculate(wheel_speeds[3], motor_4.rotor_speed, 0.01f);

        // === 4. 通过 CAN 发送给电机 ===
        bsp_can1_sendmotorcmd(motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd);

        // === 5. 周期调度 ===
        osDelay(10);
    }
}

extern "C" {
    static OmniWheelTask omniwheel_task;

    void OmniWheelTask_Init() {
        omniwheel_task.start((char*)"OmniWheelTask", 256, osPriorityNormal);
    }
}