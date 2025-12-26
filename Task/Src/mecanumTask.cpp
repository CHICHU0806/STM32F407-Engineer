//
// Created by 20852 on 2025/9/20.
//

#include "mecanumTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "mecanum.h"
#include "dbus.h"
#include "debug_vars.h"

extern DJI_motor_info motor_1;
extern DJI_motor_info motor_2;
extern DJI_motor_info motor_3;
extern DJI_motor_info motor_4;
extern DJI_motor_info motor_5;
extern remote_control_info remote_control;

//云台状态记录
extern float yaw, pitch, roll;

void MecanumTask::run() {

    for (;;){
        float Vx = remote_control.X * 0.5f;
        float Vy = -remote_control.Y * 0.5f;
        float omega = remote_control.Z * 0.5f;

        auto MotorSpeeds = Mecanum(0.5f, 0.5f, 0.076f).Mecanum_Calculate(Vx, Vy, omega);

        int16_t motor1_cmd = speed_pid_calculate(MotorSpeeds[0],motor_1.rotor_speed,0.01f);
        int16_t motor2_cmd = speed_pid_calculate(MotorSpeeds[1],motor_2.rotor_speed,0.01f);
        int16_t motor3_cmd = speed_pid_calculate(MotorSpeeds[2],motor_3.rotor_speed,0.01f);
        int16_t motor4_cmd = speed_pid_calculate(MotorSpeeds[3],motor_4.rotor_speed,0.01f);

        // float omega_feedback = mecanum_calcomega(motor_1.rotor_speed,motor_2.rotor_speed,motor_3.rotor_speed,motor_4.rotor_speed);

        //bsp_can2_djimotorcmd(motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd);
        // bsp_can2_djimotorcmdfive2eight(0,0,0,0);
        // bsp_can1_sendchassisdata(omega_feedback);

        osDelay(10);
    }
}

extern "C"{
    static MecanumTask mecanumtask;

    void MecanumTask_Init() {
        mecanumtask.start((char*)"MecanumTask", 512, osPriorityHigh);
    }
}