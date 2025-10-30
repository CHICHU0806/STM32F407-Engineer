//
// Created by 20852 on 2025/10/30.
//

#ifndef STARTM3508_USART_DECODE_H
#define STARTM3508_USART_DECODE_H

#pragma once
#include "usart_protocol.h"

#ifdef __cplusplus
struct ControlCommand {
    float yaw_target;
    float pitch_target;
    float chassis_vx;
    float chassis_vy;
    bool updated;  // 是否有新数据
};

extern ControlCommand g_control_cmd;

class UartDecoder {
public:
    explicit UartDecoder(UartProtocol& proto);

    // 解析一帧有效数据（由 UartProtocol 调用）
    void onFrame(uint8_t type, const uint8_t* data, uint8_t len);

private:
    void handleMotorCmd(const uint8_t* data, uint8_t len);
};

extern UartProtocol proto6;

#endif

#ifdef __cplusplus
extern "C"{
#endif

    void MyUartCallback(volatile uint8_t* buf, int len);
#ifdef __cplusplus
}
#endif

#endif //STARTM3508_USART_DECODE_H
