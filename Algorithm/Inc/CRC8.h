//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_CRC8_H
#define STARTM3508_CRC8_H

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    uint8_t crc8_calc(const uint8_t *data, uint16_t len, uint8_t init);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_CRC8_H