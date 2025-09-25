//
// Created by 20852 on 2025/9/25.
//

#include "../Inc/measureSpeedTask.h"
#include "bsp_can.h"
#include "speed_pid.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;
extern motor_info motor_5;

int output_1 = 0;
int output_2 = 0;
int output_3 = 0;
int output_4 = 0;
int output_5 = 0;

void MeasureSpeedTask::run() {
    float target_speed_1 = 1000.0f; // 电机1目标速度
    float target_speed_2 = 1000.0f; // 电机2目标速度
    float target_speed_3 = 1000.0f; // 电机3目标速度
    float target_speed_4 = 1000.0f; // 电机4目标速度
    //float target_speed_5 = 1000.0f; // 电机5目标速度

    speed_pid_clear(); // 清零积分

    for (;;) {
        int16_t output_1 = speed_pid_calculate(target_speed_1, motor_1.rotor_speed, 0.01f);
        int16_t output_2 = speed_pid_calculate(target_speed_2, motor_2.rotor_speed, 0.01f);
        int16_t output_3 = speed_pid_calculate(target_speed_3, motor_3.rotor_speed, 0.01f);
        int16_t output_4 = speed_pid_calculate(target_speed_4, motor_4.rotor_speed, 0.01f);
        //int16_t output_5 = speed_pid_calculate(target_speed_5, motor_5.rotor_speed, 0.01f);

        bsp_can_sendmotorcmdone2four(output_1, output_2, -output_3, -output_4);
        bsp_can_sendmotorcmdfive2eight(output_5, 1000, 0, 1000);
        osDelay(5);
    }
}

extern "C" {
    static MeasureSpeedTask measureSpeedTask;

    void MeasureSpeedTask_Init() {
        measureSpeedTask.start((char*)"MeasureSpeedTask", 256, osPriorityNormal);
    }
}
