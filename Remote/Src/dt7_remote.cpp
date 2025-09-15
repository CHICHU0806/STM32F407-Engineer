#include "dt7_remote.h"

UartDma::UartDma(UART_HandleTypeDef* huart, DecodeCallback cb)
    : huart_(huart), decode_cb_(cb), rx_data_len_(0)
{
    instance_ = this; // 只允许一个 USART3 对象
    init();
}

void UartDma::init()
{
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);
    SET_BIT(huart_->Instance->CR3, USART_CR3_DMAR);

    DMAEx_MultiBufferStart_NoIT(huart_->hdmarx,
                                (uint32_t)&huart_->Instance->DR,
                                (uint32_t)rx_buf_[0],
                                (uint32_t)rx_buf_[1],
                                UART_RX_BUF_LEN);
}

void UartDma::IRQHandler(UART_HandleTypeDef* huart)
{
    if (instance_ && instance_->callback_busy_ == 0)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
            __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
        {
            instance_->uartRxIdleCallback();
        }
    }
}

void UartDma::uartRxIdleCallback()
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

void UartDma::dmaM0RxCpltCallback()
{
    huart_->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT);
    if (decode_cb_) decode_cb_(rx_buf_[0], rx_data_len_);
}

void UartDma::dmaM1RxCpltCallback()
{
    huart_->hdmarx->Instance->CR &= ~(uint32_t)(DMA_SxCR_CT);
    if (decode_cb_) decode_cb_(rx_buf_[1], rx_data_len_);
}

HAL_StatusTypeDef UartDma::DMAEx_MultiBufferStart_NoIT(DMA_HandleTypeDef* hdma,
                                                       uint32_t SrcAddress,
                                                       uint32_t DstAddress,
                                                       uint32_t SecondMemAddress,
                                                       uint32_t DataLength)
{
    HAL_StatusTypeDef status = HAL_OK;

    if (hdma->Init.Direction == DMA_MEMORY_TO_MEMORY)
    {
        hdma->ErrorCode = HAL_DMA_ERROR_NOT_SUPPORTED;
        return HAL_ERROR;
    }

    __HAL_LOCK(hdma);

    if (HAL_DMA_STATE_READY == hdma->State)
    {
        hdma->State = HAL_DMA_STATE_BUSY;
        hdma->ErrorCode = HAL_DMA_ERROR_NONE;

        hdma->Instance->CR |= (uint32_t)DMA_SxCR_DBM;
        hdma->Instance->M1AR = SecondMemAddress;
        hdma->Instance->NDTR = DataLength;

        if (hdma->Init.Direction == DMA_MEMORY_TO_PERIPH)
        {
            hdma->Instance->PAR = DstAddress;
            hdma->Instance->M0AR = SrcAddress;
        }
        else
        {
            hdma->Instance->PAR = SrcAddress;
            hdma->Instance->M0AR = DstAddress;
        }

        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
        __HAL_DMA_ENABLE(hdma);
    }
    else
    {
        __HAL_UNLOCK(hdma);
        status = HAL_BUSY;
    }

    __HAL_UNLOCK(hdma);
    return status;
}

/* ================== C 接口实现 ================== */
static UartDma* uart3_instance = nullptr;

void Uart3_Init(UART_HandleTypeDef* huart,
                void (*decode_func)(volatile uint8_t* buf, int len))
{
    static UartDma uart3(huart, decode_func);
    uart3_instance = &uart3;
}

void Uart3_IRQHandler(UART_HandleTypeDef* huart)
{
    UartDma::IRQHandler(huart);
}