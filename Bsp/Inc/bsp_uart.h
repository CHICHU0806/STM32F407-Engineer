//
// Created by 20852 on 2025/10/14.
//

#ifndef STARTM3508_BSP_UART_H
#define STARTM3508_BSP_UART_H

#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>

#ifndef BSP_UART_RX_BUF_LEN
#define BSP_UART_RX_BUF_LEN 256
#endif

#ifndef BSP_UART_TX_QUEUE_LEN
#define BSP_UART_TX_QUEUE_LEN 512
#endif

#define BSP_UART_MAX_INST 8

#ifdef __cplusplus
class BspUart {
public:
    using DecodeCb = void(*)(volatile uint8_t* buf, int len);

    explicit BspUart(UART_HandleTypeDef* huart, DecodeCb cb = nullptr);
    void init(); // 初始化 DMA 接收

    HAL_StatusTypeDef sendDMA(const uint8_t* data, uint16_t len); // 非阻塞 DMA 发送

    // 接收队列接口
    uint16_t available() const;
    uint16_t read(uint8_t* buf, uint16_t len);

private:
    UART_HandleTypeDef* huart_;
    DecodeCb decode_cb_;

    // DMA 接收缓冲
    alignas(4) volatile uint8_t rx_buf0_[BSP_UART_RX_BUF_LEN]{0};
    alignas(4) volatile uint8_t rx_buf1_[BSP_UART_RX_BUF_LEN]{0};

    // 接收队列
    uint8_t rx_queue_[BSP_UART_RX_BUF_LEN*2]{0};
    uint16_t rx_head_ = 0;
    uint16_t rx_tail_ = 0;

    // 发送队列
    uint8_t tx_queue_[BSP_UART_TX_QUEUE_LEN]{0};
    uint16_t tx_head_ = 0;
    uint16_t tx_tail_ = 0;
    uint8_t sending_ = 0;

    void initDoubleBufferDMA();
    void handleIdleIRQ();
    void handleTxCplt();

    // 多实例管理
    static inline BspUart* instances_[BSP_UART_MAX_INST] = {nullptr};
    static int registerInstance(BspUart* inst);
    static BspUart* findInstance(UART_HandleTypeDef* huart);

public:
    static BspUart* GetInstance(UART_HandleTypeDef* huart);
    static void IRQHandler(UART_HandleTypeDef* huart);
    static void TxCpltHandler(UART_HandleTypeDef* huart); // 发送完成回调
};

#endif

// C 接口
#ifdef __cplusplus
extern "C" {
#endif

void BSP_UART_Init(UART_HandleTypeDef* huart, void (*decode_func)(volatile uint8_t* buf, int len));
HAL_StatusTypeDef BSP_UART_Send_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len);
uint16_t BSP_UART_Available(UART_HandleTypeDef* huart);
uint16_t BSP_UART_Read(UART_HandleTypeDef* huart, uint8_t* buf, uint16_t len);
void BSP_UART_IRQHandler(UART_HandleTypeDef* huart);
void BSP_UART_TxCpltHandler(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BSP_UART_H
