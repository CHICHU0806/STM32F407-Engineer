//
// Created by 20852 on 2025/9/9.
//

#include "bsp_can.h"

//外部CAN句柄 如果有更多的hcan句柄同样在这里进行定义
extern CAN_HandleTypeDef hcan1;

//BSP_CAN相关内容初始化
void bsp_can::BSP_CAN_Init()
{
    static bsp_can can;
    // 1. 配置 CAN 过滤器
    can.BSP_CAN_FilterConfig();

    // 2. 启动 CAN 外设
    // 确保你的 CAN 句柄 (hcan1, hcan2, hcan3) 已经在 CubeMX 生成的 can.c 中正确初始化。
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    // 如果有其他 CAN 外设，也在这里启动

    // 3. 激活 CAN 接收中断 (当 FIFO0 中有新消息时触发)
    if (HAL_CAN_ActivateNotification(&hcan1, 0) != HAL_OK) {
        Error_Handler(); // 激活中断失败
    }
    // 如果有其他 CAN 外设，也在这里激活中断
}

//CAN过滤器配置
void bsp_can::BSP_CAN_FilterConfig() {
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

//CAN命令发送函数
HAL_StatusTypeDef bsp_can::BSP_CAN_SendMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x200;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor1 >> 8;
    TxData[1] = motor1;
    TxData[2] = motor2 >> 8;
    TxData[3] = motor2;
    TxData[4] = motor3 >> 8;
    TxData[5] = motor3;
    TxData[6] = motor4 >> 8;
    TxData[7] = motor4;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// C接口封装
extern "C" {
    void bsp_can_init() {
        static bsp_can can;
        can.BSP_CAN_Init();
    }

    void bsp_can_filter_config() {
        static bsp_can can;
        can.BSP_CAN_FilterConfig();
    }

    HAL_StatusTypeDef bsp_can_send_motor_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        static bsp_can can;
        return can.BSP_CAN_SendMotorCmd(motor1, motor2, motor3, motor4);
    }
}