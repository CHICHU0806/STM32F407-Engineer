//
// Created by 20852 on 2025/11/11.
//

#include "../Inc/measureInfantryTwoSpeedTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "angle_pid.h"
#include "dbus.h"
#include "debug_vars.h"
#include "usart.h"
#include "usart_decode.h"
#include "usart_dma.h"
#include "message_bus.h"

extern motor_info motor_1;
extern motor_info motor_2;

SpeedPID speed_pid11(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID speed_pid22(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);

int16_t motor11_cmd = 0;
int16_t motor22_cmd = 0;

void MeasureInfantryTwoSpeedTask::run() {
    speed_pid11.Clear();
    speed_pid22.Clear();

    for (;;) {
        motor11_cmd= speed_pid11.Calculate(-5000, motor_1.rotor_speed, 0.005f);
        motor22_cmd = speed_pid22.Calculate(5000, motor_2.rotor_speed, 0.005f);

        bsp_can2_sendmotorcmd(motor11_cmd,0,0,0);

        osDelay(5);
    }
}

extern "C" {

    static MeasureInfantryTwoSpeedTask measureInfantryTwoSpeedTask;

    void MeasureInfantryTwoSpeedTask_Init() {
        measureInfantryTwoSpeedTask.start((char*)"MeasureInfantryTwoSpeedTask", 256, osPriorityNormal);
    }
}
