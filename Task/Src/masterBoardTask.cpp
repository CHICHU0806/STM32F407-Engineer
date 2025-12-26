//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"

#include <sys/_intsup.h>

#include "bsp_can.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "dbus.h"

extern DJI_motor_info motor_1;
extern DJI_motor_info motor_2;
extern DJI_motor_info motor_3;
extern DJI_motor_info motor_6;
extern DJI_motor_info LK_motor_1;

extern float yaw, pitch, roll;

extern imu_data_info imu_data_chassis;

SpeedPID pitch_speed_pid(3.0f, 0.01f, 0.001f, 3500.0f, 250.0f);

SpeedPID HeroShoot_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid3(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);

//云台pitch轴
int16_t pitch_target_speed = 0;

//英雄三摩擦
int16_t HeroShoot_target_speed1 = 0;
int16_t HeroShoot_target_speed2 = 0;
int16_t HeroShoot_target_speed3 = 0;

//发射机构推弹电机


void MasterBoardTask::run() {
    pitch_speed_pid.Clear();
    HeroShoot_speed_pid1.Clear();
    HeroShoot_speed_pid2.Clear();
    HeroShoot_speed_pid3.Clear();

    for (;;) {
        //云台pitch轴速度控制
        pitch_target_speed = pitch_speed_pid.Calculate(dbus.ch[1]*1.8f, motor_6.rotor_speed, 0.001f);

        if (pitch < -7 && pitch_target_speed > 0) {
            pitch_target_speed = 0 ;
        } else if (pitch > 43 && pitch_target_speed < 0) {
            pitch_target_speed = 0 ;
        }
        // bsp_can2_djimotorcmdfive2eight(0,pitch_target_speed,0,0);

        switch (dbus.s1) {
            case 1:
                switch (dbus.s2) {
                    //小陀螺模式
                    case 1: {
                        //bsp_can1_sendremotecontrolcmd(0,0,900, dbus.s1,dbus.s2);
                        bsp_can1_lkmotorvelocitycmd(-21950);
                        break;
                    }
                   case 2 : {
                        break;
                    }
                    case 3: {
                        break;
                    }
                    default: break;
                }
                break;
            case 2:
                switch (dbus.s2) {
                    //发射模式（左下右上）
                    //提供图传抬升，准镜旋转，pitch升降与云台yaw的旋转
                    case 1: {
                        //英雄三摩擦轮速度控制（不调试发射需要注释掉）
                        // HeroShoot_target_speed1 = HeroShoot_speed_pid1.Calculate(-6000, motor_1.rotor_speed, 0.01f);
                        // HeroShoot_target_speed2 = HeroShoot_speed_pid1.Calculate(6000, motor_2.rotor_speed, 0.01f);
                        // HeroShoot_target_speed3 = HeroShoot_speed_pid1.Calculate(-6000, motor_3.rotor_speed, 0.01f);

                        bsp_can2_djimotorcmdfive2eight(-dbus.ch[3]*2,pitch_target_speed,0,0);
                        bsp_can1_lkmotortorquecmd(-dbus.ch[0]*0.45f);
                        bsp_can2_djimotorcmd(HeroShoot_target_speed1,HeroShoot_target_speed2,HeroShoot_target_speed3,dbus.ch[2]*2);
                        break;
                    }
                    //自由控制（左下右下）
                    //提供云台与底盘的全状态分别控制
                    case 2: {
                        //bsp_can1_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4], dbus.s1,dbus.s2);
                        // bsp_can1_lkmotortorquecmd(-dbus.ch[0] * 0.45f);
                        bsp_can1_lkmotorincreposvelrestrictcmd(-dbus.ch[0],100);
                        break;
                    }
                    case 3: {
                        break;
                    }
                    default: break;
                }
                break;
            case 3:
                switch (dbus.s2) {
                    case 1: {
                        break;
                    }
                    case 2: {
                        break;
                    }
                    //云台自稳+跟随
                    //提供底盘跟随云台朝向的运动方式
                    case 3: {
                        //bsp_can1_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4], dbus.s1,dbus.s2);
                        //bsp_can1_lkmotorvelocitycmd(-dbus.ch[4] * 23-dbus.ch[0] * 13);
                        bsp_can1_lkmotortorquecmd(-dbus.ch[0]*0.45f);
                        break;
                    }
                    default: break;
                }
                break;
            default: break;
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

