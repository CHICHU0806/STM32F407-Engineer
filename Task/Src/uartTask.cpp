//
// Created by 20852 on 2025/10/14.
//

#include "uartTask.h"
#include "bsp_uart.h"
#include "usart.h"

void UARTTask::run() {
    for (;;) {

    }
}

extern "C" {
    static UARTTask uart_task;

    void UartTask_Init() {
        uart_task.start((char*)"UARTTask", 256, osPriorityNormal);
    }
}