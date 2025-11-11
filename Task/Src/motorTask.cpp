//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"
#include "usart.h"
#include "usart_decode.h"
#include "dt7_remote.h"
#include "message_bus.h"

void MotorTask::run() {

    for (;;) {
        bsp_can_sendmotorcmd(1000,1000,1000,1000);
        bsp_can_sendmotorcmdfive2eight(3000,3000,3000,30000);
        osDelay(5);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
