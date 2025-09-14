#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// === 遥控器数据结构 ===
typedef struct {
    int16_t ch[5];   // 摇杆通道
    uint8_t s[2];    // 开关位置
} RCData;

#ifdef __cplusplus
class RemoteControl {
public:
    RemoteControl(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

    void init();                       // 初始化 DMA + IDLE 中断
    void handleUartIrq();              // 在 USART3_IRQHandler 里调用
    const RCData* getData() const;     // 获取解析结果

private:
    static constexpr uint16_t FRAME_LEN = 18;
    static constexpr uint16_t BUF_LEN   = 36;

    UART_HandleTypeDef *m_huart;
    DMA_HandleTypeDef  *m_hdma;

    uint8_t m_rxBuf[BUF_LEN];  // DMA缓冲区
    RCData  m_data;            // 解码后的数据

    void parseSbus(uint8_t *buf);
};

extern RemoteControl rc;


#endif

#ifdef __cplusplus
extern "C" {
#endif

    void rc_init();
    void rc_handleuartirq();
    const RCData* rc_getdata();

#ifdef __cplusplus
}
#endif

#endif
