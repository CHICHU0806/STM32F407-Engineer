//
// Created by 20852 on 2025/11/27.
//

#ifndef STARTM3508_SPI_DECODE_H
#define STARTM3508_SPI_DECODE_H

#pragma once
#include <cstdint>

struct ImuAccel { float ax, ay, az; };
struct ImuGyro  { float gx, gy, gz; };

class SPI_Decode {
public:
    // payload: pointer to 6 bytes [XL,XH,YL,YH,ZL,ZH]
    static ImuAccel DecodeAccel(const volatile uint8_t* payload);
    static ImuGyro  DecodeGyro (const volatile uint8_t* payload);
};


#endif //STARTM3508_SPI_DECODE_H