//
// Created by 20852 on 2025/9/19.
//

#include "../Inc/motorTask.h"
#include <cstring>
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"
#include "usart.h"
#include "usart_decode.h"
#include "usart_dma.h"
#include "message_bus.h"
#include "bsp_dwt.h"
#include "ist8310driver.h"
#include "BMI088.h"

extern motor_info motor_6;
extern motor_info LK_motor_1;

SpeedPID example_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
// SpeedPID example_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
//
int16_t example_motor1_cmd = 0;
// int16_t example_motor2_cmd = 0;
void MotorTask::run() {
for (;;) {
    example_motor1_cmd = example_speed_pid1.Calculate(dbus.ch[1]*3.0f, motor_6.rotor_speed, 0.01f);

    bsp_can1_lkmotorspeedcmd(30000);

        osDelay(10);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 512, osPriorityNormal);
    }
}
