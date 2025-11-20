//
// Created by 20852 on 2025/11/20.
//

#include "bsp_dwt.h"

uint8_t DWT_Init(void)
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

static uint32_t DWT_GetCycleCount(void)
{
    return DWT->CYCCNT;
}
