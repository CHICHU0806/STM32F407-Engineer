//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_CRC16_H
#define STARTM3508_CRC16_H

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    uint16_t crc16_calc(const uint8_t *data, uint32_t len, uint16_t init);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_CRC16_H