#include "bsp_uart.h"

// --------------------------- 多实例管理 ---------------------------
int BspUart::registerInstance(BspUart* inst)
{
    for (int i = 0; i < BSP_UART_MAX_INST; ++i) {
        if (instances_[i] == nullptr) {
            instances_[i] = inst;
            return i;
        }
    }
    return -1;
}

BspUart* BspUart::findInstance(UART_HandleTypeDef* huart)
{
    for (int i = 0; i < BSP_UART_MAX_INST; ++i) {
        if (instances_[i] && instances_[i]->huart_ == huart) return instances_[i];
    }
    return nullptr;
}

BspUart* BspUart::GetInstance(UART_HandleTypeDef* huart)
{
    return findInstance(huart);
}

// --------------------------- 构造函数 ---------------------------
BspUart::BspUart(UART_HandleTypeDef* huart, DecodeCb cb)
    : huart_(huart), decode_cb_(cb)
{
    registerInstance(this);
}

// --------------------------- 初始化 DMA ---------------------------
void BspUart::init()
{
    initDoubleBufferDMA();
}

void BspUart::initDoubleBufferDMA()
{
    if (!huart_) return;

    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_UART_ENABLE_IT(huart_, UART_IT_IDLE);

    // 启动 M0 DMA
    HAL_UART_Receive_DMA(huart_, (uint8_t*)rx_buf0_, BSP_UART_RX_BUF_LEN);
    __HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);

    // 启动双缓冲 DMA
    HAL_DMAEx_MultiBufferStart(
        huart_->hdmarx,
        (uint32_t)&huart_->Instance->DR,
        (uint32_t)rx_buf0_,
        (uint32_t)rx_buf1_,
        BSP_UART_RX_BUF_LEN
    );
}

// --------------------------- 接收中断处理 ---------------------------
void BspUart::IRQHandler(UART_HandleTypeDef* huart)
{
    BspUart* inst = findInstance(huart);
    if (!inst) return;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
    {
        inst->handleIdleIRQ();
    }
}

void BspUart::handleIdleIRQ()
{
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    __HAL_DMA_DISABLE(huart_->hdmarx);

    uint32_t ndtr = huart_->hdmarx->Instance->NDTR;
    uint32_t recvd = BSP_UART_RX_BUF_LEN - ndtr;
    bool ct = (huart_->hdmarx->Instance->CR & DMA_SxCR_CT) != 0;

    volatile uint8_t* src = ct ? rx_buf1_ : rx_buf0_;

    // 写入接收队列
    for (uint32_t i = 0; i < recvd; ++i) {
        rx_queue_[rx_head_] = src[i];
        rx_head_ = (rx_head_ + 1) % (BSP_UART_RX_BUF_LEN * 2);
        if (rx_head_ == rx_tail_) {
            rx_tail_ = (rx_tail_ + 1) % (BSP_UART_RX_BUF_LEN * 2); // 丢弃溢出
        }
    }

    // 原回调
    if (decode_cb_) decode_cb_(src, (int)recvd);

    huart_->hdmarx->Instance->NDTR = BSP_UART_RX_BUF_LEN;
    __HAL_DMA_ENABLE(huart_->hdmarx);
}

// --------------------------- 发送 DMA + 队列 ---------------------------
HAL_StatusTypeDef BspUart::sendDMA(const uint8_t* data, uint16_t len)
{
    if (!huart_ || len == 0) return HAL_ERROR;

    uint16_t free_space = BSP_UART_TX_QUEUE_LEN - ((tx_head_ >= tx_tail_) ? (tx_head_ - tx_tail_) : (BSP_UART_TX_QUEUE_LEN + tx_head_ - tx_tail_));
    if (len > free_space) return HAL_BUSY;

    for (uint16_t i = 0; i < len; ++i) {
        tx_queue_[tx_head_] = data[i];
        tx_head_ = (tx_head_ + 1) % BSP_UART_TX_QUEUE_LEN;
    }

    if (!sending_) {
        uint16_t tx_len = (tx_head_ > tx_tail_) ? (tx_head_ - tx_tail_) : (BSP_UART_TX_QUEUE_LEN - tx_tail_);
        sending_ = 1;
        HAL_UART_Transmit_DMA(huart_, &tx_queue_[tx_tail_], tx_len);
    }

    return HAL_OK;
}

void BspUart::handleTxCplt()
{
    if (!sending_) return;

    uint16_t sent_len = (tx_head_ > tx_tail_) ? (tx_head_ - tx_tail_) : (BSP_UART_TX_QUEUE_LEN - tx_tail_);
    tx_tail_ = (tx_tail_ + sent_len) % BSP_UART_TX_QUEUE_LEN;

    if (tx_head_ != tx_tail_) {
        uint16_t tx_len = (tx_head_ > tx_tail_) ? (tx_head_ - tx_tail_) : (BSP_UART_TX_QUEUE_LEN - tx_tail_);
        HAL_UART_Transmit_DMA(huart_, &tx_queue_[tx_tail_], tx_len);
    } else {
        sending_ = 0;
    }
}

// --------------------------- 接收队列接口 ---------------------------
uint16_t BspUart::available() const
{
    if (rx_head_ >= rx_tail_) return rx_head_ - rx_tail_;
    return BSP_UART_RX_BUF_LEN*2 - (rx_tail_ - rx_head_);
}

uint16_t BspUart::read(uint8_t* buf, uint16_t len)
{
    uint16_t avail = available();
    if (len > avail) len = avail;

    for (uint16_t i = 0; i < len; ++i) {
        buf[i] = rx_queue_[rx_tail_];
        rx_tail_ = (rx_tail_ + 1) % (BSP_UART_RX_BUF_LEN*2);
    }
    return len;
}

// --------------------------- 静态中断分发 ---------------------------
void BspUart::TxCpltHandler(UART_HandleTypeDef* huart)
{
    BspUart* inst = findInstance(huart);
    if (inst) inst->handleTxCplt();
}

// --------------------------- C 接口 ---------------------------
extern "C" {
    static BspUart* bsp_uart_instances_ptr[BSP_UART_MAX_INST] = {nullptr};

    void BSP_UART_Init(UART_HandleTypeDef* huart, void (*decode_func)(volatile uint8_t* buf, int len))
    {
        BspUart* inst = BspUart::GetInstance(huart);
        if (!inst) {
            for (int i = 0; i < BSP_UART_MAX_INST; ++i) {
                if (!bsp_uart_instances_ptr[i]) {
                    bsp_uart_instances_ptr[i] = new BspUart(huart, decode_func);
                    bsp_uart_instances_ptr[i]->init();
                    break;
                }
            }
        }
    }

    HAL_StatusTypeDef BSP_UART_Send_DMA(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t len)
    {
        BspUart* inst = BspUart::GetInstance(huart);
        return inst ? inst->sendDMA(data, len) : HAL_ERROR;
    }

    uint16_t BSP_UART_Available(UART_HandleTypeDef* huart)
    {
        BspUart* inst = BspUart::GetInstance(huart);
        return inst ? inst->available() : 0;
    }

    uint16_t BSP_UART_Read(UART_HandleTypeDef* huart, uint8_t* buf, uint16_t len)
    {
        BspUart* inst = BspUart::GetInstance(huart);
        return inst ? inst->read(buf, len) : 0;
    }

    void BSP_UART_IRQHandler(UART_HandleTypeDef* huart)
    {
        BspUart::IRQHandler(huart);
    }

    void BSP_UART_TxCpltHandler(UART_HandleTypeDef* huart)
    {
        BspUart::TxCpltHandler(huart);
    }
}
