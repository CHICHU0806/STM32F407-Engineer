//
// Created by 20852 on 2025/11/27.
//
// spi_dma.cpp
#include "spi_dma.h"
#include "spi_decode.h"
#include <cstring>

SPI_Dma* SPI_Dma::instance_ = nullptr;

SPI_Dma::SPI_Dma(SPI_HandleTypeDef* hspi)
    : hspi_(hspi), acc_cs_port_(nullptr), acc_cs_pin_(0),
      gyr_cs_port_(nullptr), gyr_cs_pin_(0), decode_cb_(nullptr)
{
    instance_ = this;
}

SPI_Dma* SPI_Dma::Instance() {
    return instance_;
}

void SPI_Dma::Init(GPIO_TypeDef* acc_cs_port, uint16_t acc_cs_pin,
                   GPIO_TypeDef* gyr_cs_port, uint16_t gyr_cs_pin,
                   SpiDecodeCb cb)
{
    acc_cs_port_ = acc_cs_port;
    acc_cs_pin_ = acc_cs_pin;
    gyr_cs_port_ = gyr_cs_port;
    gyr_cs_pin_ = gyr_cs_pin;
    decode_cb_ = cb;

    // prepare tx buffers (cmd + zeros)
    uint8_t acc_cmd = AccReadCmd(BMI088_ACCEL_XOUT_L);
    uint8_t gyr_cmd = GyrReadCmd(BMI088_GYRO_X_L);

    acc_tx_buf_[0] = acc_cmd;
    gyr_tx_buf_[0] = gyr_cmd;
    for (int i = 1; i < FRAME_LEN; ++i) {
        acc_tx_buf_[i] = 0x00;
        gyr_tx_buf_[i] = 0x00;
    }

    // ensure CS high
    CS_High(acc_cs_port_, acc_cs_pin_);
    CS_High(gyr_cs_port_, gyr_cs_pin_);
}

// helpers
inline void SPI_Dma::CS_Low(GPIO_TypeDef* port, uint16_t pin) {
    if (port) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
inline void SPI_Dma::CS_High(GPIO_TypeDef* port, uint16_t pin) {
    if (port) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
}

inline uint8_t SPI_Dma::AccReadCmd(uint8_t reg) {
    return BMI088_ACC_READ_ADDR(reg);
}
inline uint8_t SPI_Dma::GyrReadCmd(uint8_t reg) {
    return BMI088_GYR_READ_ADDR(reg);
}

// Start single accel read (non-blocking)
HAL_StatusTypeDef SPI_Dma::StartAccelRead()
{
    if (!hspi_) return HAL_ERROR;
    if (acc_busy_) return HAL_BUSY;

    uint8_t idx = acc_next_idx_;
    // frame: tx = acc_tx_buf_, rx = acc_rx_buf_[idx]
    acc_busy_ = 1;

    CS_Low(acc_cs_port_, acc_cs_pin_);
    // ensure SPI state is ready
    // Initiate DMA TxRx: transfer FRAME_LEN bytes
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(hspi_, acc_tx_buf_, acc_rx_buf_[idx], (uint16_t)FRAME_LEN);
    if (st != HAL_OK) {
        acc_busy_ = 0;
        CS_High(acc_cs_port_, acc_cs_pin_);
    } else {
        // toggle for next time
        acc_next_idx_ = 1 - idx;
    }
    return st;
}

// Start single gyro read (non-blocking)
HAL_StatusTypeDef SPI_Dma::StartGyroRead()
{
    if (!hspi_) return HAL_ERROR;
    if (gyr_busy_) return HAL_BUSY;

    uint8_t idx = gyr_next_idx_;
    gyr_busy_ = 1;

    CS_Low(gyr_cs_port_, gyr_cs_pin_);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(hspi_, gyr_tx_buf_, gyr_rx_buf_[idx], (uint16_t)FRAME_LEN);
    if (st != HAL_OK) {
        gyr_busy_ = 0;
        CS_High(gyr_cs_port_, gyr_cs_pin_);
    } else {
        gyr_next_idx_ = 1 - idx;
    }
    return st;
}

// EXTI handlers (call these from your EXTI IRQ)
void SPI_Dma::AccelExtiHandler()
{
    // clear EXTI flag must be done by caller or here if you pass pin; keep minimal:
    // start read if not busy
    StartAccelRead();
}
void SPI_Dma::GyroExtiHandler()
{
    StartGyroRead();
}

// DMA IRQ handlers: to be called from DMA IRQ with hdma pointer
void SPI_Dma::DMA_RxCpltIRQ(DMA_HandleTypeDef* hdma)
{
    // When HAL SPI TxRx DMA completes, HAL will set hspi->State and call HAL Callback if enabled.
    // Here we will manually check which buffer was filled by comparing hdma pointer's NDTR or by tracking busy flags.
    // Simpler approach: if acc_busy_ is set -> assume accel transfer finished; else if gyr_busy_ set -> gyro finished.
    // Note: If you have both transfers simultaneously (rare), you should disambiguate via hdma pointer comparison.
    if (acc_busy_) {
        // completed accel transfer -> find which buffer holds payload (we toggled acc_next_idx_ already)
        uint8_t completed_idx = 1 - acc_next_idx_; // because we toggled on start
        // payload starts at rx[1], length PAYLOAD_LEN
        uint8_t* payload = acc_rx_buf_[completed_idx] + 1;
        // call decode
        if (decode_cb_) decode_cb_(1,payload, PAYLOAD_LEN);
        // raise CS
        CS_High(acc_cs_port_, acc_cs_pin_);
        acc_busy_ = 0;
    }
    else if (gyr_busy_) {
        uint8_t completed_idx = 1 - gyr_next_idx_;
        uint8_t* payload = gyr_rx_buf_[completed_idx] + 1;
        if (decode_cb_) decode_cb_(2,payload, PAYLOAD_LEN);
        CS_High(gyr_cs_port_, gyr_cs_pin_);
        gyr_busy_ = 0;
    } else {
        // Unknown: ignore
    }
}

// TX complete (not used much here)
void SPI_Dma::DMA_TxCpltIRQ(DMA_HandleTypeDef* hdma)
{
    // nothing by default
}

const uint8_t* SPI_Dma::GetAccelPayloadBuf(int idx) const
{
    if (idx < 0 || idx > 1) return nullptr;

    // payload = 去掉第一个 dummy/cmd 字节
    return &acc_rx_buf_[idx][1];
}

const uint8_t* SPI_Dma::GetGyroPayloadBuf(int idx) const
{
    if (idx < 0 || idx > 1) return nullptr;

    // payload = 去掉第一个 dummy/cmd 字节
    return &acc_rx_buf_[idx][1];
}

void SPI_Dma::IRQHandler_EXTI_Accel()
{
    // PC4 accel INT
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4)) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
        AccelExtiHandler();
    }
}

void SPI_Dma::IRQHandler_EXTI_Gyro()
{
    // PC5 gyro INT
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5)) {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
        GyroExtiHandler();
    }
}

void SPI_Dma::IRQHandler_DMA_Rx(DMA_HandleTypeDef* hdma)
{
    HAL_DMA_IRQHandler(hdma);  // 保持 HAL 状态一致
    DMA_RxCpltIRQ(hdma);
}

void SPI_Dma::IRQHandler_DMA_Tx(DMA_HandleTypeDef* hdma)
{
    HAL_DMA_IRQHandler(hdma);
    DMA_TxCpltIRQ(hdma);
}

extern "C" {
    void IMU_SPI_DMA_Init(SPI_HandleTypeDef* hspi)
    {
        // 创建单例对象
        static SPI_Dma spi_dma(hspi);

        // 绑定到你的 BMI088 引脚
        spi_dma.Init(
            GPIOA, GPIO_PIN_4,
            GPIOB,  GPIO_PIN_0,
            BMI088_DecodeCallback   // 你自定义的回调
        );

        // 注意：EXTI IRQ 和 DMA IRQ 你已经在 .cpp 里通过 C 接口导出了，
        // 这里只需要确保它们在 stm32xx_it.c 里被调用即可。
    }

    // EXTI PC4
    void SPI_EXTI_Accel_IRQHandler()
    {
        if (SPI_Dma::Instance())
            SPI_Dma::Instance()->IRQHandler_EXTI_Accel();
    }

    // EXTI PC5
    void SPI_EXTI_Gyro_IRQHandler()
    {
        if (SPI_Dma::Instance())
            SPI_Dma::Instance()->IRQHandler_EXTI_Gyro();
    }

    // DMA RX Stream2
    void SPI_DMA_Rx_IRQHandler(DMA_HandleTypeDef* hdma)
    {
        if (SPI_Dma::Instance())
            SPI_Dma::Instance()->IRQHandler_DMA_Rx(hdma);
    }

    // DMA TX Stream3
    void SPI_DMA_Tx_IRQHandler(DMA_HandleTypeDef* hdma)
    {
        if (SPI_Dma::Instance())
            SPI_Dma::Instance()->IRQHandler_DMA_Tx(hdma);
    }
}
