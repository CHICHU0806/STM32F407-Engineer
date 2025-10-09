//
// Created by 20852 on 2025/9/9.
//

#ifndef STARTM3508_BSP_CAN_H
#define STARTM3508_BSP_CAN_H

#pragma once
#include "can.h"

typedef struct {
    int16_t rotor_angle;      // 电机转子角度
    int16_t rotor_speed;      // 电机转子速度
    int16_t torque_current;   // 电机转矩电流
    int8_t  temp;             // 电机温度
} motor_info;

#ifdef __cplusplus
class bsp_can {
public:
    void BSP_CAN_Init();
    void BSP_CAN_FilterConfig();
    HAL_StatusTypeDef BSP_CAN_SendMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef BSP_CAN_SendMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void bsp_can_init();
    void bsp_can_filterconfig();
    HAL_StatusTypeDef bsp_can_sendmotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
    HAL_StatusTypeDef bsp_can_sendmotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_CAN_H