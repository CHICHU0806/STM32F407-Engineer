//
// Created by 20852 on 2025/11/27.
//

#include "spi_decode.h"
#include <cstdint>

// conversion factors, adjust to your chosen ranges
// typical BMI088 ±3g accel LSB ~ 16384 @ ±2g? check your range; here we use example values
static constexpr float ACC_LSB = 1.0f / 16384.0f;   // g per LSB
static constexpr float GYRO_LSB = 1.0f / 16.4f;     // deg/s per LSB (for 2000 dps)

static  int16_t le_to_i16(const uint8_t* b) {
    return (int16_t)((b[1] << 8) | b[0]);
}
ImuAccel SPI_Decode::DecodeAccel(const volatile uint8_t* payload) {
    int16_t ax = le_to_i16((const uint8_t*)payload + 0);
    int16_t ay = le_to_i16((const uint8_t*)payload + 2);
    int16_t az = le_to_i16((const uint8_t*)payload + 4);
    return { ax * ACC_LSB, ay * ACC_LSB, az * ACC_LSB };
}

ImuGyro SPI_Decode::DecodeGyro(const volatile uint8_t* payload) {
    int16_t gx = le_to_i16((const uint8_t*)payload + 0);
    int16_t gy = le_to_i16((const uint8_t*)payload + 2);
    int16_t gz = le_to_i16((const uint8_t*)payload + 4);
    return { gx * GYRO_LSB, gy * GYRO_LSB, gz * GYRO_LSB };
}
