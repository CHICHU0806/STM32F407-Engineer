//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "dbus.h"

extern motor_info motor_1;

int output = 0;

void MotorTask::run() {
    for (;;){
        int output = speed_pid_calculate((int16_t)10 * dbus.ch[1], motor_1.rotor_speed, 0.01f);
        bsp_can_sendmotorcmd(output, output, output, output);
        osDelay(10);
    }
}

extern "C" {
    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
