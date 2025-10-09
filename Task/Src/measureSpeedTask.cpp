//
// Created by 20852 on 2025/9/25.
//

#include "../Inc/measureSpeedTask.h"
#include "bsp_can.h"
#include "dbus.h"
#include "speed_pid.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;

SpeedPID speed_pid1(1.0f, 0.02f, 0.003f, 5000.0f, 200.0f);
SpeedPID speed_pid2(1.0f, 0.02f, 0.003f, 5000.0f, 200.0f);
SpeedPID speed_pid3(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID speed_pid4(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);

int16_t motor1_cmd = 0;
int16_t motor2_cmd = 0;
int16_t motor3_cmd = 0;
int16_t motor4_cmd = 0;

void MeasureSpeedTask::run() {;
    speed_pid1.Clear();
    speed_pid2.Clear();
    speed_pid3.Clear();
    speed_pid4.Clear();

    for (;;) {

        motor1_cmd = speed_pid1.Calculate(2000, motor_1.rotor_speed, 0.005f);
        motor2_cmd = speed_pid2.Calculate(-2000, motor_2.rotor_speed, 0.005f);
        motor3_cmd = speed_pid3.Calculate(4700, motor_3.rotor_speed, 0.005f);
        motor4_cmd = speed_pid4.Calculate(-4700, motor_4.rotor_speed, 0.005f);

        bsp_can_sendmotorcmd(motor1_cmd,motor2_cmd,motor3_cmd,motor4_cmd);

        osDelay(5);
    }
}

extern "C" {
    static MeasureSpeedTask measureSpeedTask;

    void MeasureSpeedTask_Init() {
        measureSpeedTask.start((char*)"MeasureSpeedTask", 256, osPriorityNormal);
    }
}
