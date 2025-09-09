//
// Created by 20852 on 2025/9/9.
//

#ifndef STARTM3508_BSP_CAN_H
#define STARTM3508_BSP_CAN_H

#pragma once
#include "can.h"

#ifdef __cplusplus
class bsp_can {
public:
    void BSP_CAN_Init();
    void BSP_CAN_FilterConfig();
    HAL_StatusTypeDef BSP_CAN_SendMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

void bsp_can_init();
void bsp_can_filter_config();
HAL_StatusTypeDef bsp_can_send_motor_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_CAN_H