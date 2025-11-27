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

typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
    uint8_t s1;
    uint8_t s2;
}  remote_control_info;

#ifdef __cplusplus
class bsp_can {
public:
    void bsp_can_init();
    void BSP_CAN_FilterConfig();
    HAL_StatusTypeDef BSP_CAN2_SendMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef BSP_CAN2_SendMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef BSP_CAN2_SendMotorCmdNine2Eleven(int16_t motor9,int16_t motor10,int16_t motor11);
    HAL_StatusTypeDef BSP_CAN1_SendRemoteControlCmd(int16_t X,int16_t Y,int16_t Z, uint8_t s1,uint8_t s2);

    HAL_StatusTypeDef BSP_CAN1_LKMotorCloseCmd();
    HAL_StatusTypeDef BSP_CAN1_LKMotorStartCmd();
    HAL_StatusTypeDef BSP_CAN1_LKMotorCurrentCmd(int16_t current);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BSP_CAN_Init();

    //DJI Motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can2_sendmotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef bsp_can2_sendmotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
    HAL_StatusTypeDef bsp_can2_sendmotorcmdnine2eleven(int16_t motor9,int16_t motor10,int16_t motor11);
    HAL_StatusTypeDef bsp_can1_sendremotecontrolcmd(int16_t X,int16_t Y,int16_t Z,uint8_t s1,uint8_t s2);

    //LK motor CAN发送函数接口
    HAL_StatusTypeDef bsp_can1_lkmotorclosecmd();
    HAL_StatusTypeDef bsp_can1_lkmotorstartcmd();
    HAL_StatusTypeDef bsp_can1_lkmotorcurrentcmd(int16_t current);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_CAN_H