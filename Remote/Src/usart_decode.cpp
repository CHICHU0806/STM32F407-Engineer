//
// Created by 20852 on 2025/10/30.
//

#include "usart_decode.h"

#include "dt7_remote.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "message_bus.h"
#include "debug_vars.h"


UartProtocol proto6(nullptr);
UartDecoder decoder6(proto6);
bool motor_cmd_triggered = false;

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
    motor_cmd_triggered = true;
    if (len == 0 ) return;
    // //处理电机命令
    float speed = 0.0f;
    memcpy(&speed, data, sizeof(float));
    debug_P = speed; // 用于调试观察
    motorCmd.target_speed = speed;
    debug_I = motorCmd.target_speed;
}

extern "C" {
    void MyUartCallback(volatile uint8_t* buf, int len)
    {
        proto6.feedDMABuffer(buf, len); // 将 DMA 缓冲区数据逐字节送给协议解析
    }
}