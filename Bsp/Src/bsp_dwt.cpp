//
// Created by 20852 on 2025/11/20.
//

#include "bsp_dwt.h"

uint8_t BSP_DWT_Init(void)
{
    // 1. 使能 DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 2. 复位计数器
    DWT->CYCCNT = 0;

    // 3. 启动 CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 4. 检查是否正常运行（CYCCNT 是否变化）
    uint32_t c1 = DWT->CYCCNT;
    uint32_t c2 = DWT->CYCCNT;

    return (c2 == c1);  // 0=成功, 1=失败
}

void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * us;

    while ((DWT->CYCCNT - start) < cycles);
}

void DWT_Delay_ms(uint32_t ms)
{
    while (ms--)
        DWT_Delay_us(1000);
}

 uint32_t DWT_GetCycleCount(void)
{
    return DWT->CYCCNT;
}

uint64_t DWT_GetMicroseconds(void)
{
    // 当前 DWT 计数器值
    uint32_t cycles = DWT->CYCCNT;

    // CPU 主频（一般是 400MHz）
    uint32_t cpu_freq = HAL_RCC_GetHCLKFreq();

    // cycles / (freq / 1e6) = 时间（us）
    return (uint64_t)cycles * 1000000ull / cpu_freq;
}

float DWT_GetSeconds(void)
{
    // 获取微秒并转成秒
    return (DWT_GetMicroseconds() * 1e-6f);
}

