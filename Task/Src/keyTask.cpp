//
// Created by 20852 on 2025/10/15.
//

#include "../Inc/keyTask.h"

#include "bsp_can.h"
#include "speed_pid.h"
#include "usart.h"

extern motor_info motor_5;

void KeyTask::run() {;

    for (;;) {
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET)
        {
            int16_t pushout = speed_pid_calculate(750, motor_5.rotor_speed, 0.01f);
            bsp_can_sendmotorcmdfive2eight(pushout, 0, 0, 0);
            osDelay(10);
        }
        else {
            int16_t pushout = speed_pid_calculate(-750, motor_5.rotor_speed, 0.01f);
            bsp_can_sendmotorcmdfive2eight(pushout, 0, 0, 0);
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