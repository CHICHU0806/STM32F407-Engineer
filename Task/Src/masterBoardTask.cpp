//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"

#include <sys/_intsup.h>

#include "bsp_can.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "dbus.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_6;
extern motor_info LK_motor_1;

extern float yaw, pitch, roll;

extern imu_data_info imu_data_chassis;

SpeedPID pitch_speed_pid(3.0f, 0.05f, 0.005f, 9000.0f, 400.0f);

SpeedPID HeroShoot_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid3(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);

//云台pitch轴
int16_t pitch_target_speed = 0;

//英雄三摩擦
int16_t HeroShoot_target_speed1 = 0;
int16_t HeroShoot_target_speed2 = 0;
int16_t HeroShoot_target_speed3 = 0;

bool yaw_initialized = false;
float yaw_initial = 0.0f;

void MasterBoardTask::run() {
    pitch_speed_pid.Clear();
    HeroShoot_speed_pid1.Clear();
    HeroShoot_speed_pid2.Clear();
    HeroShoot_speed_pid3.Clear();

    for (;;) {
        //云台pitch轴速度控制
        pitch_target_speed = pitch_speed_pid.Calculate(dbus.ch[1]*3.0f, motor_6.rotor_speed, 0.001f);

        bsp_can2_sendmotorcmdfive2eight(0,pitch_target_speed,0,0);

        //英雄三摩擦轮速度控制
        // HeroShoot_target_speed1 = HeroShoot_speed_pid1.Calculate(-5000, motor_1.rotor_speed, 0.01f);
        // HeroShoot_target_speed2 = HeroShoot_speed_pid1.Calculate(5000, motor_2.rotor_speed, 0.01f);
        // HeroShoot_target_speed3 = HeroShoot_speed_pid1.Calculate(-5000, motor_3.rotor_speed, 0.01f);
        //
        // bsp_can2_sendmotorcmd(HeroShoot_target_speed1,HeroShoot_target_speed2,HeroShoot_target_speed3,0);

        switch (dbus.s1) {
            case 1:
                switch (dbus.s2) {
                    case 1: {
                        //小陀螺模式
                        bsp_can1_sendremotecontrolcmd(0,0,660, dbus.s1,dbus.s2);
                        bsp_can1_lkmotorspeedcmd(-660 * 23-dbus.ch[0] * 13);
                        break;
                    }
                    default: break;
                }
                break;
            case 2:
                switch (dbus.s2) {
                    case 2: {
                        //自由控制
                        bsp_can1_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4], dbus.s1,dbus.s2);
                        bsp_can1_lkmotorspeedcmd(-dbus.ch[0] * 13);
                        break;
                    }
                }
                break;
            case 3:
                switch (dbus.s2) {
                    case 3: {
                        //云台自稳
                        bsp_can1_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4], dbus.s1,dbus.s2);
                        if (!yaw_initialized) {
                            yaw_initial = imu_data_chassis.yaw; // 直接取底盘发过来的 yaw yaw_initialized = true;
                        }
                        bsp_can1_lkmotorspeedcmd(-dbus.ch[4] * 23-dbus.ch[0] * 13);
                        break;
                    }
                    default: break;
                }
                break;
            default: break;
        }

        if (dbus.s1 != 3 || dbus.s2 != 3) {
            yaw_initialized = false; // 重置初始化标志
        }

        osDelay(1);
    }
}

extern "C" {
    static MasterBoardTask master_board_task;

    void MasterBoardTask_Init() {
        master_board_task.start((char*)"MasterBoardTask", 1024, osPriorityHigh);
    }

}

