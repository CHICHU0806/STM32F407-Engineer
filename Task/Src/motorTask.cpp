//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;
extern motor_info motor_4;
extern motor_info motor_5;

int angle_output = 0;
int speed_output = 1000;

void MotorTask::run() {
    int32_t target_angle = 0;
    angle_pid_clear();
    speed_pid_clear();

    for (;;) {
        target_angle= dbus.ch[0]*(8191/1320.0f); // 目标角度，单位：0.1度，范围-8191~8191
        angle_output = angle_pid_calculate(target_angle, motor_5.rotor_angle, 0.005f);
        speed_output = speed_pid_calculate(angle_output, motor_5.rotor_speed, 0.005f);
        bsp_can_sendmotorcmd(500, 500, 500, 500);
        bsp_can_sendmotorcmdfive2eight(speed_output, speed_output, speed_output, speed_output);
        osDelay(5);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
