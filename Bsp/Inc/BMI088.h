//
// Created by 20852 on 2025/11/17.
//

#ifndef STARTM3508_BMI088_H
#define STARTM3508_BMI088_H

#pragma once
#include "can.h"
#include "semphr.h"

#ifdef __cplusplus
class Bmi088 {
public:
    Bmi088(BspSpiDMA& spiGyro,
    GPIO_TypeDef* csGyroPort, uint16_t csGyroPin);

    bool init();

    // 在 1kHz 任务中调用，发起一次读取
    bool requestReadGyro();

    // DMA 完成后回调会更新 gyro[]
    float gyro[3]; // rad/s

    // 信号量，DMA 完成时 give
    SemaphoreHandle_t dmaDone;

    // 单例指针，供全局回调使用
    static Bmi088* instance;

private:
    BspSpiDMA& spiGyro_;
    GPIO_TypeDef* csGyroPort_;
    uint16_t csGyroPin_;

    uint8_t txBuf_[7]; // addr + 6 bytes
    uint8_t rxBuf_[7];

    // 在 DMA 完成回调上下文中被调用
    void onGyroDMAComplete();
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BMI088_Gyro_DMACompleteCallback(BspSpiDMA* spi);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_BMI088_H