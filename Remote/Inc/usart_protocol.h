//
// Created by 20852 on 2025/10/30.
//

#ifndef STARTM3508_USART_PROTOCOL_H
#define STARTM3508_USART_PROTOCOL_H

#pragma once
#include "stdint.h"

#ifdef __cplusplus
#include <functional>

class UartProtocol
{
public:
    static constexpr uint8_t FRAME_HEAD_1 = 0xFE;
    static constexpr uint8_t FRAME_HEAD_2 = 0xEF;
    static constexpr uint8_t FRAME_MAX_LEN = 64;

    using FrameCallback = std::function<void(uint8_t frame_type, const uint8_t* payload, uint8_t len)>;

    explicit UartProtocol(FrameCallback cb);

    void setCallback(FrameCallback cb) { cb_ = std::move(cb); }

    // 供 DMA 或中断层调用 — 逐字节输入接收到的数据
    void input(uint8_t byte);

    // 计算校验
    static uint8_t calcChecksum(const uint8_t* data, uint8_t len);

    // 发送封装函数：将数据打包成帧
    uint8_t buildFrame(uint8_t* out_buf, uint8_t frame_type, const uint8_t* payload, uint8_t len);

    // 供 DMA 解码函数调用 — 批量输入接收到的数据
    void feedDMABuffer(volatile uint8_t* buf, int len) {
        for (int i = 0; i < len; ++i) {
            input(buf[i]);
        }
    }

private:
    enum ParseState {
        WAIT_HEAD1,
        WAIT_HEAD2,
        WAIT_TYPE,
        WAIT_LEN,
        WAIT_PAYLOAD,
        WAIT_CHECKSUM
    };

    ParseState state_ = WAIT_HEAD1;
    uint8_t recv_buf_[FRAME_MAX_LEN];
    FrameCallback cb_ = nullptr;
    uint8_t frame_type_;
    uint8_t data_len_;
    uint8_t data_index_;
};
#endif

#endif //STARTM3508_USART_PROTOCOL_H