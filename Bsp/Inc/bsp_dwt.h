//
// Created by 20852 on 2025/11/20.
//

#ifndef STARTM3508_BSP_DWT_H
#define STARTM3508_BSP_DWT_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

    // 初始化 DWT 计数器，返回 0=成功, 1=失败
    uint8_t DWT_Init(void);

    // 微秒级延时
    void DWT_Delay_us(uint32_t us);

    // 毫秒级延时（使用 us 延时实现）
    void DWT_Delay_ms(uint32_t ms);

    // 获取当前 CYCCNT（CPU 时钟计数）
    static uint32_t DWT_GetCycleCount(void);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_DWT_H