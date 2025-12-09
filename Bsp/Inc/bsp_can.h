//
// Created by 20852 on 2025/9/9.
//

#ifndef STARTM3508_BSP_CAN_H
#define STARTM3508_BSP_CAN_H

#pragma once
#include "can.h"

typedef struct {
    //M3508，GM6020，M2006电机协议反馈信息
    int16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度

    uint16_t last_angle;      // 上次角度
    int16_t  total_angle;     // 累计角度
    int32_t  round_count;     // 圈数计数
    uint8_t  inited;          // 是否初始化标志
} motor_info;

typedef struct
{
    uint8_t  id;           // 电机 ID (0~15)
    uint8_t  err;          // 电机错误码

    uint16_t pos_raw;      // 原始位置（12bit/16bit根据型号）
    uint16_t vel_raw;      // 原始速度（12bit）
    uint16_t torque_raw;   // 原始力矩（12bit）

    uint8_t  temp_mos;     // MOS温度

    float pos;             // 物理量（可选，自动算）
    float vel;             // 物理量（可选）
    float torque;          // 物理量（可选）

} DMMotor_t;

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
    uint8_t s1;
    uint8_t s2;
}  remote_control_info;

typedef struct {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} imu_data_info;

#ifdef __cplusplus
class bsp_can {
public:
    void bsp_can_init();
    void BSP_CAN_FilterConfig();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef BSP_CAN2_DJIMotorCmdNine2Eleven(int16_t motor9,int16_t motor10,int16_t motor11);

    //DM Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN2_DMMotorDisableCmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef BSP_CAN2_DMMotorEnableCmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef BSP_CAN2_DMMotorPositionCmd(int16_t ID, float position, float velocity);
    HAL_StatusTypeDef BSP_CAN2_DMMotorVelocityCmd(int16_t ID, float velocity);

    //LK Motor CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_LKMotorCloseCmd();
    HAL_StatusTypeDef BSP_CAN1_LKMotorStartCmd();
    HAL_StatusTypeDef BSP_CAN1_LKMotorTorqueCmd(int16_t current);
    HAL_StatusTypeDef BSP_CAN1_LKMotorVelocityCmd(int32_t velocity);

    //遥控器数据 CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_SendRemoteControlCmd(int16_t X,int16_t Y,int16_t Z, uint8_t s1,uint8_t s2);

    //IMU数据 CAN发送函数接口
    HAL_StatusTypeDef BSP_CAN1_SendIMUData(int16_t roll, int16_t pitch, int16_t yaw);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BSP_CAN_Init();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can2_djimotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef bsp_can2_djimotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef bsp_can2_djimotorcmdnine2eleven(int16_t motor9,int16_t motor10,int16_t motor11);

    //DM Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can2_dmmotordisablecmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef bsp_can2_dmmotorenablecmd(uint16_t ID, uint16_t mode);
    HAL_StatusTypeDef bsp_can2_dmmotorpositioncmd(int16_t ID, float position, float velocity);
    HAL_StatusTypeDef bsp_can2_dmmotorvelocitycmd(int16_t ID, float velocity);

    //LK motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_lkmotorclosecmd();
    HAL_StatusTypeDef bsp_can1_lkmotorstartcmd();
    HAL_StatusTypeDef bsp_can1_lkmotortorquecmd(int16_t torque);
    HAL_StatusTypeDef bsp_can1_lkmotorvelocitycmd(int32_t velocity);

    //遥控器数据 CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_sendremotecontrolcmd(int16_t X,int16_t Y,int16_t Z,uint8_t s1,uint8_t s2);

    //IMU数据 CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_sendimudata(int16_t roll, int16_t pitch, int16_t yaw);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_CAN_H