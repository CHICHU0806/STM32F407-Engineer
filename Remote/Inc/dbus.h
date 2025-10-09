//
// Created by 20852 on 2025/9/15.
//

#ifndef STARTM3508_DBUS_H
#define STARTM3508_DBUS_H

#pragma once
#include <stdint.h>

typedef struct
{
    int16_t ch[5];   // 四个通道（摇杆数据）
    uint8_t s1;      // 开关 S1
    uint8_t s2;      // 开关 S2

    struct {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t l;  // 左键
        uint8_t r;  // 右键
    } mouse;

    uint16_t key;    // 键盘按键
} DBUS_t;

extern DBUS_t dbus;

void DBUS_Decode(volatile uint8_t* buf, int len);

#ifdef __cplusplus
extern "C" {
#endif

    void dbus_decode(volatile uint8_t* buf, int len);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_DBUS_H