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


void MotorTask::run() {
    static uint8_t frame[16];
    uint8_t payload[3] = {0x55, 0x66, 0x77};
    for (;;) {
        uint8_t len = proto6.buildFrame(frame, 0x02, payload, 3);
        Uart_Transmit_DMA(&huart6, frame, len);
        osDelay(500);
    }
}

extern "C" {

    static MotorTask motorTask;

    void MotorTask_Init() {
        motorTask.start((char*)"MotorTask", 256, osPriorityNormal);
    }
}
