//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "cascade_pid.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;
extern motor_info motor_4;

int angle_output = 5000;
int speed_output = 0;

void MotorTask::run() {
    const int32_t target_angle = 4096;
    angle_pid_clear();
    speed_pid_clear();

    for (;;) {
        //int16_t angle_output = angle_pid_calculate(target_angle, motor_1.rotor_angle, 0.01f);
        speed_output = speed_pid_calculate(angle_output, motor_1.rotor_speed, 0.005f);
        bsp_can_sendmotorcmd((int16_t)speed_output, (int16_t)speed_output, (int16_t)speed_output, (int16_t)speed_output);
        osDelay(5);
    }
}

extern "C" {
    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
