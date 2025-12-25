//
// Created by 20852 on 2025/10/30.
//

#include "usart_decode.h"

#include "usart_dma.h"
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
    if (len < 2) return; // 至少需要2字节表示int16_t

    // 小端序解析：低位在前
    auto raw_speed = static_cast<int16_t>(data[0] | (data[1] << 8));

    // 转换为 float（方便后续控制）
    float speed = raw_speed;

    // 更新全局调试与控制变量
    debug_roll = speed;
    motorCmd.target_speed = speed;
    debug_pitch = motorCmd.target_speed;
}

extern "C" {
    void MyUartCallback(volatile uint8_t* buf, int len)
    {
        proto6.feedDMABuffer(buf, len); // 将 DMA 缓冲区数据逐字节送给协议解析
    }
}