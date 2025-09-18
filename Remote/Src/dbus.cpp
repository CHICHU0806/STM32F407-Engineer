//
// Created by 20852 on 2025/9/15.
//

#include "dbus.h"
#include <string.h>

DBUS_t dbus;

void DBUS_Decode(volatile uint8_t* buf, int len)
{
    if (len != 18) return; // DBUS 一帧固定 18 字节

    // 0000_0111_1111_1111
    dbus.ch[0] = ((buf[0] | (buf[1] << 8)) & 0x07FF) - 1024;
    dbus.ch[1] = (((buf[1] >> 3) | (buf[2] << 5)) & 0x07FF) - 1024;
    dbus.ch[2] = (((buf[2] >> 6) | (buf[3] << 2) | (buf[4] << 10)) & 0x07FF) - 1024;
    dbus.ch[3] = (((buf[4] >> 1) | (buf[5] << 7)) & 0x07FF) - 1024;

    dbus.s1 = ((buf[5] >> 4) & 0x000C) >> 2;  // S1
    dbus.s2 = ((buf[5] >> 4) & 0x0003);       // S2

    dbus.mouse.x = buf[6] | (buf[7] << 8);
    dbus.mouse.y = buf[8] | (buf[9] << 8);
    dbus.mouse.z = buf[10] | (buf[11] << 8);

    dbus.mouse.l = buf[12];
    dbus.mouse.r = buf[13];

    dbus.key = buf[14] | (buf[15] << 8);
}


extern "C" {
    void dbus_decode(volatile uint8_t *buf, int len) {
        DBUS_Decode(buf, len);
    }
}

