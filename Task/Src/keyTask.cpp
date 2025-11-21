//
// Created by 20852 on 2025/10/15.
//

#include "../Inc/keyTask.h"

#include "bsp_can.h"
#include "speed_pid.h"
#include "usart.h"

extern motor_info motor_5;

SpeedPID key_speed_pid(1.5f, 0.01f, 0.001f, 5000.0f, 200.0f);

void KeyTask::run() {;

    for (;;) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            float pushout = key_speed_pid.Calculate(1500, motor_5.rotor_speed, 0.01f);
            bsp_can2_sendmotorcmdfive2eight(pushout, 0, 0, 0);
            osDelay(10);
        }
        else {
            float pushout = key_speed_pid.Calculate(-850, motor_5.rotor_speed, 0.01f);
            bsp_can2_sendmotorcmdfive2eight(pushout, 0, 0, 0);
            osDelay(10);
        }
    }
}

extern "C" {
    static KeyTask key_task;

    void KeyTask_Init() {
        key_task.start((char*)"KeyTask", 256, osPriorityNormal);
    }
}