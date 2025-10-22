//
// Created by 20852 on 2025/10/17.
//

#include "../Inc/swerveTask.h"
#include "bsp_can.h"
#include "speed_pid.h"
#include "swerve.h"
#include "dbus.h"
#include "debug_vars.h"
#include "angle_pid.h"
#include <cmath>

// ---- 四个驱动电机 (M3508) ----
extern motor_info motor_1;
extern motor_info motor_2;
extern motor_info motor_3;
extern motor_info motor_4;

// ---- 四个舵向电机 (GM6020) ----'
extern motor_info motor_5;
extern motor_info motor_6;
extern motor_info motor_7;
extern motor_info motor_8;

//舵pid
AnglePID Servo_pid(15.0f, 0.8f, 0.08f, 5000.0f, 5000.0f);

void SwerveTask::run() {
    for (;;) {
        //——————————舵机部分——————————
        float ref_angle = motor_6.total_angle;  // 以左前轮为基准

        // 固定偏差（根据标定）
        const int32_t diff_rl = -3450;
        const int32_t diff_fr = -1353;
        const int32_t diff_rr = +1402;

        int32_t target_rl = ref_angle + diff_rl;
        int32_t target_fl = ref_angle + dbus.ch[0]*0.4;
        int32_t target_fr = ref_angle + diff_fr;
        int32_t target_rr = ref_angle + diff_rr;

        // ---------- 外环（角度环）输出目标速度 ----------
        float target_speed_rl = Servo_pid.Calculate(target_rl, motor_5.total_angle, 0.005f);
        float target_speed_fl = Servo_pid.Calculate(target_fl, motor_6.total_angle, 0.005f);
        float target_speed_fr = Servo_pid.Calculate(target_fr, motor_7.total_angle, 0.005f);
        float target_speed_rr = Servo_pid.Calculate(target_rr, motor_8.total_angle, 0.005f);

        // ---------- 内环（速度环）输出控制电流 ----------
        int16_t cmd_rl = speed_pid_calculate(target_speed_rl, motor_5.rotor_speed, 0.005f);
        int16_t cmd_fl = speed_pid_calculate(target_speed_fl, motor_6.rotor_speed, 0.005f);
        int16_t cmd_fr = speed_pid_calculate(target_speed_fr, motor_7.rotor_speed, 0.005f);
        int16_t cmd_rr = speed_pid_calculate(target_speed_rr, motor_8.rotor_speed, 0.005f);


        //——————————电机部分——————————
        int16_t V = dbus.ch[3]* 3;  // 目标速度

        int16_t drive_cmd_rl = speed_pid_calculate(-V, motor_1.rotor_speed, 0.005f);
        int16_t drive_cmd_fl = speed_pid_calculate(-V, motor_2.rotor_speed, 0.005f);
        int16_t drive_cmd_fr = speed_pid_calculate(-V, motor_3.rotor_speed, 0.005f);
        int16_t drive_cmd_rr = speed_pid_calculate(-V, motor_4.rotor_speed, 0.005f);


        // ---------- 发送 ----------
         bsp_can_sendmotorcmdfive2eight(cmd_rl, cmd_fl, cmd_fr, cmd_rr);
         bsp_can_sendmotorcmd(drive_cmd_rl, drive_cmd_fl, drive_cmd_fr, drive_cmd_rr);

        osDelay(5);
    }
}

extern "C" {
    static SwerveTask swerve_task;

    void SwerveTask_Init() {
        swerve_task.start((char*)"SwerveTask", 256, osPriorityNormal);
    }
}
