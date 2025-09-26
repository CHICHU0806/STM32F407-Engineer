//
// Created by 20852 on 2025/9/9.
//

#include "bsp_can.h"
#include "debug_vars.h"

//外部CAN句柄 如果有更多的hcan句柄同样在这里进行定义
extern CAN_HandleTypeDef hcan1;

//创建不同电机的结构体变量
motor_info motor_1;
motor_info motor_2;
motor_info motor_3;
motor_info motor_4;
motor_info motor_5;

//BSP_CAN相关内容初始化
void bsp_can::BSP_CAN_Init()
{
    static bsp_can can;
    // 1. 配置 CAN 过滤器
    can.BSP_CAN_FilterConfig();

    // 2. 启动 CAN 外设
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    // 如果有其他 CAN 外设，也在这里启动

    // 3. 激活 CAN 接收中断 (当 FIFO0 中有新消息时触发)
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
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
    TxHeader.StdId = 0x200; //标准标识符
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

HAL_StatusTypeDef bsp_can::BSP_CAN_SendMotorCmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x1FF; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor5 >> 8;
    TxData[1] = motor5;
    TxData[2] = motor6 >> 8;
    TxData[3] = motor6;
    TxData[4] = motor7 >> 8;
    TxData[5] = motor7;
    TxData[6] = motor8 >> 8;
    TxData[7] = motor8;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // 在这里解析RxData，更新电机状态等
        if(hcan->Instance == CAN1)
        {
            switch(RxHeader.StdId)
            {
                case 0x201://此处仅接收了id为0x201电机的报文
                {
                    motor_1.rotor_angle    = ((RxData[0] << 8) | RxData[1]);
                    motor_1.rotor_speed    = ((RxData[2] << 8) | RxData[3]);
                    motor_1.torque_current = ((RxData[4] << 8) | RxData[5]);
                    motor_1.temp           =   RxData[6];

                    debug_angle = motor_1.rotor_speed;
                    break;
                }
                case 0x202: {
                    motor_2.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_2.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_2.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_2.temp          =   RxData[6];
                    break;
                }
                case 0x203: {
                    motor_3.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_3.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_3.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_3.temp          =   RxData[6];
                    break;
                }
                case 0x204: {
                    motor_4.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_4.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_4.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_4.temp          =   RxData[6];
                    break;
                }
                default: ;
            }
        }
    }
}

// C接口封装
extern "C" {
    static bsp_can can;

    void bsp_can_init() {
        can.BSP_CAN_Init();
    }

    void bsp_can_filterconfig() {
        can.BSP_CAN_FilterConfig();
    }

    HAL_StatusTypeDef bsp_can_sendmotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        return can.BSP_CAN_SendMotorCmd(motor1, motor2, motor3, motor4);
    }

    HAL_StatusTypeDef bsp_can_sendmotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
        return can.BSP_CAN_SendMotorCmdfive2eight(motor5, motor6, motor7, motor8);
    }
}