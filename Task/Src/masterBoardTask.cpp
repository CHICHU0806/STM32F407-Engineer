//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"
#include "bsp_can.h"
#include "angle_pid.h"
#include "speed_pid.h"
#include "dbus.h"

extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_6;

SpeedPID pitch_speed_pid(3.0f, 0.05f, 0.005f, 9000.0f, 400.0f);
SpeedPID HeroShoot_speed_pid1(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid2(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);
SpeedPID HeroShoot_speed_pid3(1.5f, 0.08f, 0.003f, 9000.0f, 250.0f);

int16_t pitch_target_speed = 0;
int16_t HeroShoot_target_speed1 = 0;
int16_t HeroShoot_target_speed2 = 0;
int16_t HeroShoot_target_speed3 = 0;

void MasterBoardTask::run() {
    pitch_speed_pid.Clear();
    HeroShoot_speed_pid1.Clear();
    HeroShoot_speed_pid2.Clear();
    HeroShoot_speed_pid3.Clear();

    for (;;) {
        pitch_target_speed = pitch_speed_pid.Calculate(dbus.ch[1]*3.0f, motor_6.rotor_speed, 0.01f);
        HeroShoot_target_speed1 = HeroShoot_speed_pid1.Calculate(-3000, motor_1.rotor_speed, 0.01f);
        HeroShoot_target_speed2 = HeroShoot_speed_pid1.Calculate(3000, motor_2.rotor_speed, 0.01f);
        HeroShoot_target_speed3 = HeroShoot_speed_pid1.Calculate(-3000, motor_3.rotor_speed, 0.01f);

        bsp_can1_sendremotecontrolcmd(dbus.ch[3],dbus.ch[2],dbus.ch[4]);
        bsp_can1_lkmotorcurrentcmd(-dbus.ch[0]*0.5f);
        bsp_can2_sendmotorcmdfive2eight(0,pitch_target_speed,0,0);
        bsp_can2_sendmotorcmd(HeroShoot_target_speed1,HeroShoot_target_speed2,HeroShoot_target_speed3,0);
        osDelay(10);
    }
}

extern "C" {
    static MasterBoardTask master_board_task;

    void MasterBoardTask_Init() {
        master_board_task.start((char*)"MasterBoardTask", 256, osPriorityNormal);
    }

}

