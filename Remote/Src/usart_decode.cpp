//
// Created by 20852 on 2025/10/30.
//

#include "usart_decode.h"

#include "dt7_remote.h"
#include "usart.h"
#include "stdio.h"

UartProtocol proto6(nullptr);
UartDecoder decoder6(proto6);

UartDecoder::UartDecoder(UartProtocol& proto) {
    // 注册回调，让协议层在解出帧后自动调用 onFrame
    proto.setCallback([this](uint8_t type, const uint8_t* data, uint8_t len) {;
        this->onFrame(type, data, len);
    });
}

void UartDecoder::onFrame(uint8_t type, const uint8_t* data, uint8_t len)
{
    switch (type)
    {
        case 0x01: handleMotorCmd(data, len); break;

        default: break;
    }
}

void UartDecoder::handleMotorCmd(const uint8_t* data, uint8_t len)
{
    if (len == 0 ) return;
    uint8_t frame[32];  // 足够大以容纳回显数据
    uint8_t sendLen = proto6.buildFrame(frame, 0x81, data, len); // 0x81 表示回显类型

    Uart_Transmit_DMA(&huart6, frame, sendLen);  // 通过串口6发送
}

extern "C" {
    void MyUartCallback(volatile uint8_t* buf, int len)
    {
        proto6.feedDMABuffer(buf, len); // 将 DMA 缓冲区数据逐字节送给协议解析
    }
}