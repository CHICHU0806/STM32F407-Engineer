//
// Created by 20852 on 2025/9/20.
//

#include "chassisTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "mecanum.h"
#include "dbus.h"
#include "debug_vars.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;

void ChassisTask::run() {
    for (;;){
        float Vx = dbus.ch[3] * 0.5f;
        float Vy = -dbus.ch[2] * 0.5f;
        float omega = -dbus.ch[0] * 0.5f;

        auto MotorSpeeds = Mecanum(0.5f, 0.5f, 0.076f).Mecanum_Calculate(Vx, Vy, omega);

        int16_t motor1_cmd = speed_pid_calculate(MotorSpeeds[0],motor_1.rotor_speed,0.01f);
        int16_t motor2_cmd = speed_pid_calculate(MotorSpeeds[1],motor_2.rotor_speed,0.01f);
        int16_t motor3_cmd = speed_pid_calculate(MotorSpeeds[2],motor_3.rotor_speed,0.01f);
        int16_t motor4_cmd = speed_pid_calculate(MotorSpeeds[3],motor_4.rotor_speed,0.01f);

        bsp_can_sendmotorcmdone2four(motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd);

        osDelay(10);
    }
}

extern "C"{
    static ChassisTask chassistask;

    void ChassisTask_Init() {
        chassistask.start((char*)"ChassisTask", 256, osPriorityNormal);
    }
}