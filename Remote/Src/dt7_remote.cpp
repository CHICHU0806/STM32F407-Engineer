#include "dt7_remote.h"
#include "debug_vars.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

RemoteControl rc(&huart3, &hdma_usart3_rx);

RemoteControl::RemoteControl(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
    : m_huart(huart), m_hdma(hdma)
{
    for(int i=0; i<5; i++) m_data.ch[i] = 0;
    m_data.s[0] = m_data.s[1] = 0;
}

void RemoteControl::init()
{
    // 开启 DMA 接收
    HAL_UART_Receive_DMA(m_huart, m_rxBuf, BUF_LEN);
    // 打开 IDLE 中断
    __HAL_UART_ENABLE_IT(m_huart, UART_IT_IDLE);
}

void RemoteControl::handleUartIrq()
{
    if(__HAL_UART_GET_FLAG(m_huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);

        HAL_UART_DMAStop(m_huart);
        uint16_t len = BUF_LEN - __HAL_DMA_GET_COUNTER(m_hdma);

        if(len == FRAME_LEN)
        {
            parseSbus(m_rxBuf);
        }

        HAL_UART_Receive_DMA(m_huart, m_rxBuf, BUF_LEN);
    }
}

const RCData* RemoteControl::getData() const
{
    return &m_data;
}

void RemoteControl::parseSbus(uint8_t *sbus_buf)
{
    m_data.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF;
    m_data.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;
    m_data.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF;
    m_data.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;
    m_data.ch[4] = ((sbus_buf[5] >> 4) | (sbus_buf[6] << 4)) & 0x07FF;

    m_data.s[0] = (sbus_buf[7] >> 0) & 0x03;
    m_data.s[1] = (sbus_buf[7] >> 2) & 0x03;
}

extern "C"{
    static RemoteControl rc_c(&huart3, &hdma_usart3_rx);

    void rc_init() {
        rc_c.init();
    }

    void rc_handleuartirq() {
        rc_c.handleUartIrq();
    }

    const RCData* rc_getdata() {
        return rc_c.getData();
    }
}