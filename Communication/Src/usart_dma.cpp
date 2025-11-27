#include "usart_dma.h"

UsartDma::UsartDma(UART_HandleTypeDef* huart, DecodeCallback cb)
    : huart_(huart), decode_cb_(cb), rx_data_len_(0)
{
    // 注册到实例表，失败时可断言或打印错误
    int idx = registerInstance(this);
    (void)idx;

    Init();
}

int UsartDma::registerInstance(UsartDma* inst) {
    for (int i = 0; i < DT7DMA_MAX_INSTANCES; ++i) {
        if (instances_[i] == nullptr) {
            instances_[i] = inst;
            return i;
        }
    }
    return -1; // 表已满
}

UsartDma* UsartDma::findInstance(UART_HandleTypeDef* huart) {
    if (!huart) {
        return nullptr;
    }
    for (int i = 0; i < DT7DMA_MAX_INSTANCES; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) return instances_[i];
    }
    return nullptr;
}

void UsartDma::Init()
{
    /* ========= 接收 DMA 初始化 ========= */
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);

    DMAEx_MultiBufferStart_NoIT(huart_->hdmarx,
                                (uint32_t)&huart_->Instance->DR,
                                (uint32_t)rx_buf_[0],
                                (uint32_t)rx_buf_[1],
                                UART_RX_BUF_LEN);

    /* ========= 发送 DMA 初始化 ========= */
    SET_BIT(huart_->Instance->CR3, USART_CR3_DMAT);  // 允许 DMA 发送
}

void UsartDma::IRQHandler(UART_HandleTypeDef* huart)
{
    UsartDma* inst = findInstance(huart);
    if (!inst) {
        return;
    }

    //接收
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        if (inst->callback_busy_ == 0) {
            inst->uartRxIdleCallback();
        }
    }

    //发送
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC))
    {
        inst->uartTxCpltCallback();
    }
}

HAL_StatusTypeDef UsartDma::Transmit_DMA(const uint8_t* data, uint16_t len)
{
    if (tx_busy_) return HAL_BUSY;
    tx_busy_ = 1;
    return HAL_UART_Transmit_DMA(huart_, data, len);
}

void UsartDma::uartRxIdleCallback()
{
    callback_busy_ = 1;

    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_DMA_DISABLE(huart_->hdmarx);

    rx_data_len_ = UART_RX_BUF_LEN - huart_->hdmarx->Instance->NDTR;

    if (huart_->hdmarx->Instance->CR & DMA_SxCR_CT)
        dmaM1RxCpltCallback();
    else
        dmaM0RxCpltCallback();

    __HAL_DMA_SET_COUNTER(huart_->hdmarx, UART_RX_BUF_LEN);
    __HAL_DMA_ENABLE(huart_->hdmarx);

    callback_busy_ = 0;
}

void UsartDma::uartTxCpltCallback()
{
    tx_busy_ = 0;
}

void UsartDma::dmaM0RxCpltCallback()
{
    huart_->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
    if (decode_cb_) decode_cb_(rx_buf_[0], rx_data_len_);
}

void UsartDma::dmaM1RxCpltCallback()
{
    huart_->hdmarx->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
    if (decode_cb_) decode_cb_(rx_buf_[1], rx_data_len_);
}

HAL_StatusTypeDef UsartDma::DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef* hdma,
                                                       uint32_t SrcAddress,
                                                       uint32_t DstAddress,
                                                       uint32_t SecondMemAddress,
                                                       uint32_t DataLength)
{
    if (!hdma) return HAL_ERROR;

    // MEM->MEM 模式不支持
    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
        hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
        return HAL_ERROR;
    }

    __HAL_LOCK(hdma);

    HAL_StatusTypeDef status = HAL_OK;

    if (hdma->State != HAL_DMA_STATE_READY)
    {
        __HAL_UNLOCK(hdma);
        return HAL_BUSY;
    }

    // 设置 DMA 状态
    hdma->State = HAL_DMA_STATE_BUSY;
    hdma->ErrorCode = HAL_DMA_ERROR_NONE;

    // 启用双缓冲模式
    hdma->Instance->CR |= DMA_SxCR_DBM;

    // 设置两个内存缓冲区地址
    hdma->Instance->M0AR = DstAddress;
    hdma->Instance->M1AR = SecondMemAddress;

    // 设置传输大小
    hdma->Instance->NDTR = DataLength;

    // 设置外设地址
    if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
    {
        hdma->Instance->PAR = DstAddress;  // 外设寄存器地址
        hdma->Instance->M0AR = SrcAddress; // 内存缓冲区 0
    }
    else // PERIPH->MEM
    {
        hdma->Instance->PAR = SrcAddress;  // 外设寄存器地址
        hdma->Instance->M0AR = DstAddress; // 内存缓冲区 0
    }

    // 清除传输完成标志
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));

    // 启动 DMA
    __HAL_DMA_ENABLE(hdma);

    __HAL_UNLOCK(hdma);

    return status;
}

/* ================== C 接口实现 ================== */
extern "C"{
    void Uart_Init(UART_HandleTypeDef* huart, void (*decode_func)(volatile uint8_t* buf, int len))
    {
        new UsartDma(huart, decode_func);
    }

    void Uart_IRQHandler(UART_HandleTypeDef* huart)
    {
        UsartDma::IRQHandler(huart);
    }

    HAL_StatusTypeDef Uart_Transmit_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len)
    {
        UsartDma* inst = UsartDma::GetInstance(huart);

        if (!inst) {
            return HAL_ERROR;
        }

        return inst->Transmit_DMA(data, len);
    }
}
