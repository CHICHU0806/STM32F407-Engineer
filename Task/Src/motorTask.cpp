//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"

void MotorTask::run() {

    for (;;) {
        bsp_can_sendmotorcmdnine2eleven(5000, 5000, 1000);
        osDelay(10);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
