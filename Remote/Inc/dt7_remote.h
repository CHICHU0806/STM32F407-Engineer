#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#pragma once
#include "main.h"
#include <stdint.h>

#ifndef UART_RX_BUF_LEN
#define UART_RX_BUF_LEN 256
#endif

#ifdef __cplusplus
class DT7Dma
{
public:
    using DecodeCallback = void (*)(volatile uint8_t* buf, int len);

    DT7Dma(UART_HandleTypeDef* huart, DecodeCallback cb);

    // IRQHandler 在 USART3_IRQHandler() 中调用
    static void IRQHandler(UART_HandleTypeDef* huart);

private:
    UART_HandleTypeDef* huart_;
    DecodeCallback decode_cb_;  // 回调函数指针

    volatile uint8_t rx_buf_[2][UART_RX_BUF_LEN] = {0};
    int rx_data_len_;
    uint8_t callback_busy_ = 0;

    static inline DT7Dma* instance_ = nullptr; // 只支持 USART3 单例

    void Remote_Init();
    void uartRxIdleCallback();

    void dmaM0RxCpltCallback();
    void dmaM1RxCpltCallback();

    static HAL_StatusTypeDef DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef* hdma,
                                                         uint32_t SrcAddress,
                                                         uint32_t DstAddress,
                                                         uint32_t SecondMemAddress,
                                                         uint32_t DataLength);
};
#endif

/* --------- 对 C 文件暴露的接口 --------- */
#ifdef __cplusplus
extern "C" {
#endif

    // 初始化 USART3 DMA 双缓冲接收
    void Uart3_Init(UART_HandleTypeDef* huart,
                    void (*decode_func)(volatile uint8_t* buf, int len));

    // IRQHandler，供 stm32f4xx_it.c 调用
    void Uart3_IRQHandler(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif
