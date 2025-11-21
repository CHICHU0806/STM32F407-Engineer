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
#include "BMI088driver.h"
#include "ist8310driver.h"
//
// SpeedPID example_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
// SpeedPID example_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
//
// int16_t example_motor1_cmd = 0;
// int16_t example_motor2_cmd = 0;

void MotorTask::run() {

    bsp_can1_lkmotorclosecmd();
    bsp_can1_lkmotorstartcmd();
    for (;;) {
        // example_motor1_cmd = example_speed_pid1.Calculate(3000, 0, 0.005f);
        // example_motor2_cmd = example_speed_pid2.Calculate(3000, 0, 0.005f);
        //
        // bsp_can1_sendmotorcmd(example_motor1_cmd,example_motor2_cmd,0,0);
        bsp_can1_lkmotorcurrentcmd(1000);
        osDelay(1);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 512, osPriorityRealtime);
    }
}
