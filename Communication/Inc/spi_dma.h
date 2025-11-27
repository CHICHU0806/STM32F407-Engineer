//
// Created by 20852 on 2025/11/27.
//

#ifndef STARTM3508_SPI_DMA_H
#define STARTM3508_SPI_DMA_H

#pragma once
// spi_dma.h
// SPI DMA wrapper for BMI088 (mode A: task/EXTI trigger -> single DMA transaction)
// - SPI1_RX -> DMA2_Stream2, SPI1_TX -> DMA2_Stream3 (CubeMX configured)
// - double-buffer (internal toggling), no malloc
// - manual IRQ handlers (call these from your IRQ vectors)

#include "main.h"
#include "BMI088reg.h"
#include <stdint.h>

#ifdef __cplusplus
using SpiDecodeCb = void(*)(int type, const uint8_t* payload, uint16_t len);

class SPI_Dma {
public:
    // ctor not doing hardware init; call Init() after HAL MX init
    SPI_Dma(SPI_HandleTypeDef* hspi);

    // init with CS pins and decode callback
    void Init(GPIO_TypeDef* acc_cs_port, uint16_t acc_cs_pin,
              GPIO_TypeDef* gyr_cs_port, uint16_t gyr_cs_pin,
              SpiDecodeCb cb);

    // Start a single DMA read of accel (called from imuTask or EXTI)
    HAL_StatusTypeDef StartAccelRead();

    // Start a single DMA read of gyro (called from imuTask or EXTI)
    HAL_StatusTypeDef StartGyroRead();

    // To be called from EXTI IRQ (manual)
    void AccelExtiHandler();
    void GyroExtiHandler();

    // To be called from DMA IRQ handlers (manual)
    // Pass the DMA handle pointer (e.g. &hspi1.hdmarx)
    void DMA_RxCpltIRQ(DMA_HandleTypeDef* hdma);
    void DMA_TxCpltIRQ(DMA_HandleTypeDef* hdma);

    // utility getters for raw buffers (payload pointers)
    const uint8_t* GetAccelPayloadBuf(int idx) const; // idx = 0 or 1 returns internal buffer pointer
    const uint8_t* GetGyroPayloadBuf(int idx) const;

    // singleton accessor (optional)
    static SPI_Dma* Instance();

    void IRQHandler_EXTI_Accel();
    void IRQHandler_EXTI_Gyro();
    void IRQHandler_DMA_Rx(DMA_HandleTypeDef* hdma);
    void IRQHandler_DMA_Tx(DMA_HandleTypeDef* hdma);

private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* acc_cs_port_;
    uint16_t acc_cs_pin_;
    GPIO_TypeDef* gyr_cs_port_;
    uint16_t gyr_cs_pin_;
    SpiDecodeCb decode_cb_;

    // sizes: 1 byte cmd + N payload (payload_len)
    static constexpr int PAYLOAD_LEN = 6;        // accel/gyro X_L..Z_H
    static constexpr int FRAME_LEN = 1 + PAYLOAD_LEN; // total bytes transfered

    // double buffers (no malloc)
    alignas(4) uint8_t acc_rx_buf_[2][FRAME_LEN];
    alignas(4) uint8_t acc_tx_buf_[FRAME_LEN];

    alignas(4) uint8_t gyr_rx_buf_[2][FRAME_LEN];
    alignas(4) uint8_t gyr_tx_buf_[FRAME_LEN];

    // which buffer will be used for next transaction (index toggling)
    volatile uint8_t acc_next_idx_ = 0;
    volatile uint8_t gyr_next_idx_ = 0;

    // busy flags
    volatile uint8_t acc_busy_ = 0;
    volatile uint8_t gyr_busy_ = 0;

    // internal helpers
    inline void CS_Low(GPIO_TypeDef* port, uint16_t pin);
    inline void CS_High(GPIO_TypeDef* port, uint16_t pin);

    // build read command bytes
    static inline uint8_t AccReadCmd(uint8_t reg);
    static inline uint8_t GyrReadCmd(uint8_t reg);

    // singleton
    static SPI_Dma* instance_;
};
#endif

#ifdef __cplusplus
extern "C"{
#endif
    void IMU_SPI_DMA_Init(SPI_HandleTypeDef* hspi);

    void SPI_EXTI_Accel_IRQHandler();
    void SPI_EXTI_Gyro_IRQHandler();
    void SPI_DMA_Rx_IRQHandler(DMA_HandleTypeDef* hdma);
    void SPI_DMA_Tx_IRQHandler(DMA_HandleTypeDef* hdma);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_SPI_DMA_H