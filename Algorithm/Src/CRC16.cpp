//
// Created by 20852 on 2025/10/17.
//

#include "CRC16.h"

uint16_t crc16_calc(const uint8_t *data, uint32_t len, uint16_t init)
{
    uint16_t crc = init;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
