//
// Created by 20852 on 2025/11/27.
//

#ifndef STARTM3508_SPI_DECODE_H
#define STARTM3508_SPI_DECODE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    // ---------------- IMU 数据结构 ----------------
    typedef struct {
        float ax, ay, az;
        float gx, gy, gz;
        float temperature;
    } IMU_Data_t;

    // 全局 IMU 数据（在 cpp 中定义）
    extern IMU_Data_t g_imu_data;

    // ---------------- 回调类型 ----------------
#define IMU_TYPE_ACCEL  1
#define IMU_TYPE_GYRO   2

    // SPI_Dma 将调用如下回调
    void BMI088_DecodeCallback(int type, const uint8_t* payload, uint16_t len);


#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus

class SPI_Decode
{
public:
    static float BytesToF32_Accel(const uint8_t* p);
    static float BytesToF32_Gyro (const uint8_t* p);

    static void DecodeAccel(const uint8_t* payload);
    static void DecodeGyro (const uint8_t* payload);
};
#endif

#endif //STARTM3508_SPI_DECODE_H