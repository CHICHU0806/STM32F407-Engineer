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

extern motor_info motor_1;

void MotorTask::run() {
    int output = 0;
    for (;;) {
        output = speed_pid_calculate(motorCmd.target_speed,motor_1.rotor_speed ,0.05);
        bsp_can_sendmotorcmd(output,0,0,0);
        osDelay(5);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
