//
// Created by 20852 on 2025/9/9.
//

#include "bsp_can.h"

#include <concepts>

#include "debug_vars.h"

//外部CAN句柄 如果有更多的hcan句柄同样在这里进行定义
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//创建不同电机的结构体变量
motor_info motor_1;
motor_info motor_2;
motor_info motor_3;
motor_info motor_4;
motor_info motor_5;
motor_info motor_6;
motor_info motor_7;
motor_info motor_8;
motor_info motor_9;

remote_control_info remote_control;

//BSP_CAN相关内容初始化
void bsp_can::bsp_can_init()
{
    static bsp_can can;
    // 1. 配置 CAN 过滤器
    can.BSP_CAN_FilterConfig();

    // 2. 启动 CAN 外设
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        Error_Handler(); // 启动失败，进入错误处理
    }
    // 如果有其他 CAN 外设，也在这里启动

    // 3. 激活 CAN 接收中断 (当 FIFO0 中有新消息时触发)
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler(); // 激活中断失败
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler(); // 激活中断失败
    }
    // 如果有其他 CAN 外设，也在这里激活中断
}

//CAN过滤器配置
void bsp_can::BSP_CAN_FilterConfig()
{
    CAN_FilterTypeDef filter;

    /** ------------ CAN1：过滤器 0-13 全部接收 ------------ */
    filter.FilterActivation       = ENABLE;
    filter.FilterMode             = CAN_FILTERMODE_IDMASK;
    filter.FilterScale            = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment   = CAN_FILTER_FIFO0;

    filter.FilterIdHigh           = 0x0000;
    filter.FilterIdLow            = 0x0000;
    filter.FilterMaskIdHigh       = 0x0000;
    filter.FilterMaskIdLow        = 0x0000;

    filter.FilterBank             = 0;       // CAN1 的第一个过滤器组
    filter.SlaveStartFilterBank   = 14;      // 14 之后给 CAN2 用

    HAL_CAN_ConfigFilter(&hcan1, &filter);


    /** ------------ CAN2：过滤器 14-27 全部接收 ------------ */
    filter.FilterBank = 14;                 // CAN2 的第一个过滤器组
    HAL_CAN_ConfigFilter(&hcan1, &filter);  // 注意：必须用 hcan1 配置 CAN2 过滤器
}

//CAN命令发送函数
HAL_StatusTypeDef bsp_can::BSP_CAN2_SendMotorCmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
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
    return HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN2_SendMotorCmdFive2Eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
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

HAL_StatusTypeDef bsp_can::BSP_CAN2_SendMotorCmdNine2Eleven(int16_t motor9,int16_t motor10,int16_t motor11) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x2FF; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = motor9 >> 8;
    TxData[1] = motor9;
    TxData[2] = motor10 >> 8;
    TxData[3] = motor10;
    TxData[4] = motor11 >> 8;
    TxData[5] = motor11;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN1_SendRemoteControlCmd(int16_t X,int16_t Y,int16_t Z,uint8_t s1,uint8_t s2) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x301; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = X >> 8;
    TxData[1] = X;
    TxData[2] = Y >> 8;
    TxData[3] = Y;
    TxData[4] = Z >> 8;
    TxData[5] = Z;
    TxData[6] = s1;
    TxData[7] = s2;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorCloseCmd() {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x141; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0x80;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorStartCmd() {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x141; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0x88;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorTorqueCmd(int16_t current) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x141; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA1;   // 命令字：力矩控制
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;

    TxData[4] = (uint8_t)(current & 0xFF);         // 力矩低字节
    TxData[5] = (uint8_t)((current >> 8) & 0xFF);  // 力矩高字节

    TxData[6] = 0x00;
    TxData[7] = 0x00;

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

HAL_StatusTypeDef bsp_can::BSP_CAN1_LKMotorSpeedCmd(int32_t speed) {
    //三要素：帧头，数据，邮箱
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    //帧头组成
    TxHeader.StdId = 0x141; //标准标识符
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    //数据填充
    TxData[0] = 0xA2;   // 命令字：力矩控制
    TxData[1] = 0x00;
    TxData[2] = 0x00;
    TxData[3] = 0x00;

    TxData[4] = (uint8_t)(speed & 0xFF);          // 最低字节
    TxData[5] = (uint8_t)((speed >> 8) & 0xFF);   // 次低字节
    TxData[6] = (uint8_t)((speed >> 16) & 0xFF);  // 次高字节
    TxData[7] = (uint8_t)((speed >> 24) & 0xFF);  // 最高字节

    //将信息推送到邮箱
    return HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

//辅助函数：更新电机累计角度
void update_motor_total_angle(motor_info* motor, uint16_t new_ecd)
{
    // 首次初始化处理
    if (!motor->inited) {
        motor->last_angle = new_ecd;
        motor->round_count = 0;
        motor->total_angle = (int32_t)new_ecd;
        motor->inited = 1;
        return;
    }

    // 使用有符号计算差分，避免 uint16 溢出问题
    int32_t diff = (int32_t)new_ecd - (int32_t)motor->last_angle;

    // 过零检测 (当差值超过半圈认为跨了圈)
    if (diff > 4096) {
        // new_ecd 小， last 大 -> 实际向上跨过 0 点， round_count 增加
        motor->round_count--;
    }
    else if (diff < -4096) {
        motor->round_count++;
    }

    // 计算连续编码器读数（ticks）
    motor->total_angle = motor->round_count * (int32_t)8192 + (int32_t)new_ecd;

    // 最后更新 last_angle
    motor->last_angle = new_ecd;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        // 在这里解析RxData，更新电机状态等
        if(hcan->Instance == CAN2)
        {
            switch(RxHeader.StdId)
            {
                case 0x201://此处仅接收了id为0x201电机的报文
                {
                    motor_1.rotor_angle    = ((RxData[0] << 8) | RxData[1]);
                    motor_1.rotor_speed    = ((RxData[2] << 8) | RxData[3]);
                    motor_1.torque_current = ((RxData[4] << 8) | RxData[5]);
                    motor_1.temp           =   RxData[6];
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
                case 0x205: {
                    motor_5.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_5.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_5.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_5.temp          =   RxData[6];
                    update_motor_total_angle(&motor_5, motor_5.rotor_angle);

                    debug_angle1 = motor_5.total_angle;
                    break;
                }
                case 0x206: {
                    motor_6.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_6.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_6.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_6.temp          =   RxData[6];
                    update_motor_total_angle(&motor_6, motor_6.rotor_angle);

                    debug_angle2 = motor_6.total_angle;
                    break;
                }
                case 0x207: {
                    motor_7.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_7.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_7.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_7.temp          =   RxData[6];
                    update_motor_total_angle(&motor_7, motor_7.rotor_angle);

                    debug_angle3 = motor_7.total_angle;
                    break;
                }
                case 0x208: {
                    motor_8.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_8.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_8.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_8.temp          =   RxData[6];
                    update_motor_total_angle(&motor_8, motor_8.rotor_angle);

                    debug_angle4 = motor_8.total_angle;
                    break;
                }
                case 0x209: {
                    motor_9.rotor_angle   = ((RxData[0] << 8) | RxData[1]);
                    motor_9.rotor_speed   = ((RxData[2] << 8) | RxData[3]);
                    motor_9.torque_current= ((RxData[4] << 8) | RxData[5]);
                    motor_9.temp          =   RxData[6];
                    break;
                }
                default: break;
            }
        }
        else if(hcan->Instance == CAN1) {
            switch(RxHeader.StdId) {
                case 0x301: {
                    // 处理遥控器数据
                    remote_control.X = (RxData[0] << 8) | RxData[1];
                    remote_control.Y = (RxData[2] << 8) | RxData[3];
                    remote_control.Z = (RxData[4] << 8) | RxData[5];
                    remote_control.s1 = RxData[6];
                    remote_control.s2 = RxData[7];
                    break;
                }
                default: break;
            }
        }
    }
}

// C接口封装
extern "C" {
    static bsp_can can;

    void BSP_CAN_Init() {
        can.bsp_can_init();
    }

    HAL_StatusTypeDef bsp_can2_sendmotorcmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4) {
        return can.BSP_CAN2_SendMotorCmd(motor1, motor2, motor3, motor4);
    }

    HAL_StatusTypeDef bsp_can2_sendmotorcmdfive2eight(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8) {
        return can.BSP_CAN2_SendMotorCmdFive2Eight(motor5, motor6, motor7, motor8);
    }

    HAL_StatusTypeDef bsp_can2_sendmotorcmdnine2eleven(int16_t motor9,int16_t motor10,int16_t motor11) {
        return can.BSP_CAN2_SendMotorCmdNine2Eleven(motor9,motor10,motor11);
    }

    HAL_StatusTypeDef bsp_can1_sendremotecontrolcmd(int16_t X,int16_t Y,int16_t Z, uint8_t s1, uint8_t s2) {
        return can.BSP_CAN1_SendRemoteControlCmd(X,Y,Z, s1, s2);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorclosecmd() {
        return can.BSP_CAN1_LKMotorCloseCmd();
    }

    HAL_StatusTypeDef bsp_can1_lkmotorstartcmd() {
        return can.BSP_CAN1_LKMotorStartCmd();
    }

    HAL_StatusTypeDef bsp_can1_lkmotortorquecmd(int16_t torque) {
        return can.BSP_CAN1_LKMotorTorqueCmd(torque);
    }

    HAL_StatusTypeDef bsp_can1_lkmotorspeedcmd(int32_t speed) {
        return can.BSP_CAN1_LKMotorSpeedCmd(speed);
    }
}