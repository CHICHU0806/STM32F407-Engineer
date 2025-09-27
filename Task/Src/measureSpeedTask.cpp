//
// Created by 20852 on 2025/9/25.
//

#include "../Inc/measureSpeedTask.h"
#include "bsp_can.h"
#include "dbus.h"
#include "speed_pid.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;

void MeasureSpeedTask::run() {
    speed_pid_clear(); // 清零积分

    for (;;) {
        int16_t abc = 15;

        int16_t motor1_cmd = speed_pid_calculate(1000, motor_1.rotor_speed, 0.01f);
        int16_t motor2_cmd = speed_pid_calculate(1000, motor_2.rotor_speed, 0.01f);
        int16_t motor3_cmd = speed_pid_calculate(1000, motor_3.rotor_speed, 0.01f);
        int16_t motor4_cmd = speed_pid_calculate(1000, motor_4.rotor_speed, 0.01f);

        bsp_can_sendmotorcmd(abc * motor1_cmd,abc*motor2_cmd,abc*motor3_cmd,abc*motor4_cmd);

        osDelay(10);
    }
}

extern "C" {
    static MeasureSpeedTask measureSpeedTask;

    void MeasureSpeedTask_Init() {
        measureSpeedTask.start((char*)"MeasureSpeedTask", 256, osPriorityNormal);
    }
}
