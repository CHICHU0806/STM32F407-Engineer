//
// Created by 20852 on 2025/10/28.
//

#include "../Inc/targetCarTask.h"

#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_4;
extern motor_info motor_5;

int angle_output1 = 0;
int speed_output1 = 0;
int angle_output2 = 0;
int speed_output2 = 0;

SpeedPID speed_pid_up(0.5f, 0.01f, 0.001f, 5000.0f, 200.0f);
SpeedPID speed_pid_down(0.5f, 0.01f, 0.001f, 5000.0f, 200.0f);
AnglePID angle_pid_up(5.0f, 0.0f, 0.00f, 5000.0f, 5000.0f);
AnglePID angle_pid_down(5.0f, 0.0f, 0.0f, 5000.0f, 5000.0f);

void TargetCarTask::run() {
    int32_t target_angle1 = 0;
    int32_t target_angle2 = 0;
    speed_pid_down.Clear();
    angle_pid_down.Clear();
    speed_pid_up.Clear();
    angle_pid_up.Clear();

    for (;;) {
        target_angle1= dbus.ch[0]*(8191/1320.0f); // 目标角度，单位：0.1度，范围-8191~8191
        target_angle2= -dbus.ch[2]*(8191/1320.0f);

        speed_output1 = speed_pid_down.Calculate(target_angle1,motor_1.rotor_speed, 0.005f);

        speed_output2 = speed_pid_up.Calculate(target_angle2, motor_2.rotor_speed, 0.005f);
        bsp_can_sendmotorcmd(speed_output1, speed_output2, 0, 0);
        osDelay(10);
    }
}

extern "C" {

    static TargetCarTask target_car_task;

    void TargetCarTask_Init() {
        target_car_task.start((char*)"TargetCarTask", 256, osPriorityNormal);
    }
}
