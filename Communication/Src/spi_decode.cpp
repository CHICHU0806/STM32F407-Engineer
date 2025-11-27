#include "spi_decode.h"

// ----------------------
// 全局 IMU 数据存储
// ----------------------
IMU_Data_t g_imu_data = {};

// ----------------------
// 内部工具
// ----------------------
static  int16_t le_i16(const uint8_t* b)
{
    return (int16_t)((b[1] << 8) | b[0]);
}

// 量程对应实际转化（可按你的配置修改）
static constexpr float ACC_LSB  = 1.0f / 16384.0f;   // g/LSB
static constexpr float GYRO_LSB = 1.0f / 16.4f;      // dps/LSB

// ======================================================
// SPI_Decode 类实现
// ======================================================

float SPI_Decode::BytesToF32_Accel(const uint8_t* p)
{
    return (float)le_i16(p) * ACC_LSB;
}

float SPI_Decode::BytesToF32_Gyro(const uint8_t* p)
{
    return (float)le_i16(p) * GYRO_LSB;
}

void SPI_Decode::DecodeAccel(const uint8_t* payload)
{
    g_imu_data.ax = BytesToF32_Accel(payload + 0);
    g_imu_data.ay = BytesToF32_Accel(payload + 2);
    g_imu_data.az = BytesToF32_Accel(payload + 4);
}

void SPI_Decode::DecodeGyro(const uint8_t* payload)
{
    g_imu_data.gx = BytesToF32_Gyro(payload + 0);
    g_imu_data.gy = BytesToF32_Gyro(payload + 2);
    g_imu_data.gz = BytesToF32_Gyro(payload + 4);
}

// ======================================================
// ***** 统一的解码回调（SPI_Dma 调用） *****
// ======================================================

extern "C"
void BMI088_DecodeCallback(int type, const uint8_t* payload, uint16_t len)
{
    (void)len; // BMI088 固定长度，无需判断

    if (type == IMU_TYPE_ACCEL)
    {
        SPI_Decode::DecodeAccel(payload);
    }
    else if (type == IMU_TYPE_GYRO)
    {
        SPI_Decode::DecodeGyro(payload);
    }
}
