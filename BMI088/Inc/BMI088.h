//
// Created by 20852 on 2025/11/29.
//

#ifndef STARTM3508_BMI088_H
#define STARTM3508_BMI088_H

#pragma once
#include "main.h"
#include <stdint.h>
#include "BMI088config.h"

#ifdef __cplusplus
class BMI088 {
public:
    struct RawData {
        uint8_t status;
        int16_t accel[3];
        int16_t temp;
        int16_t gyro[3];
    };

    struct RealData {
        uint8_t status;
        float accel[3];
        float temp;
        float gyro[3];
        float time;
    };

    //枚举错误表，用来返回错误信息
    enum
    {
        BMI088_NO_ERROR = 0x00,
        BMI088_ACC_PWR_CTRL_ERROR = 0x01,
        BMI088_ACC_PWR_CONF_ERROR = 0x02,
        BMI088_ACC_CONF_ERROR = 0x03,
        BMI088_ACC_SELF_TEST_ERROR = 0x04,
        BMI088_ACC_RANGE_ERROR = 0x05,
        BMI088_INT1_IO_CTRL_ERROR = 0x06,
        BMI088_INT_MAP_DATA_ERROR = 0x07,
        BMI088_GYRO_RANGE_ERROR = 0x08,
        BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
        BMI088_GYRO_LPM1_ERROR = 0x0A,
        BMI088_GYRO_CTRL_ERROR = 0x0B,
        BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
        BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

        BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
        BMI088_SELF_TEST_GYRO_ERROR = 0x40,
        BMI088_NO_SENSOR = 0xFF,
    };

    BMI088(SPI_HandleTypeDef* hspi,
           GPIO_TypeDef* cs_accel_port, uint16_t cs_accel_pin,
           GPIO_TypeDef* cs_gyro_port, uint16_t cs_gyro_pin);

    uint8_t init();
    bool accelInit();
    bool gyroInit();

    void read(float gyro[3], float accel[3], float *temperate);        // 读取 raw_ 并转换到 real_

    const RealData& getRealData() const { return real_; }
    const RawData&  getRawData()  const { return raw_;}

private:
    // ====== SPI 工具函数 ======
    //拉高或者拉低片选引脚，用于信息的选择接收
    void csAccelLow();
    void csAccelHigh();
    void csGyroLow();
    void csGyroHigh();

    //写单个寄存器，读单个或者多个寄存器的相关函数
    inline void accelWriteReg(uint8_t reg, uint8_t data);
    inline uint8_t accelReadReg(uint8_t reg);
    inline void accelReadMulti(uint8_t reg, uint8_t *data, uint16_t len);

    inline void gyroWriteReg(uint8_t reg, uint8_t data);
    inline uint8_t gyroReadReg(uint8_t reg);
    inline void gyroReadMulti(uint8_t reg, uint8_t *data, uint16_t len);

    static inline uint8_t BMI088_readandwrite_byte(uint8_t txdata);
    static void BMI088_read_multiple_reg(uint8_t reg, uint8_t *data, uint8_t len);
    static HAL_StatusTypeDef BMI088_read_multiple_reg_dma(uint8_t reg, uint8_t *buf, uint8_t len);

    //配置状态检查数组写入函数
    //主要用于写入精度范围，量程等内容，并且返回是否成功
    static uint8_t writeAccelConfig[BMI088_WRITE_ACCEL_REG_NUM][3];
    static uint8_t writeGyroConfig[BMI088_WRITE_GYRO_REG_NUM][3];

    //读取加速度计,陀螺仪和温度计的原始数据
    void readRawData();

    //数据转换函数
    void convertData();

    // ====== 硬件资源 ======
    SPI_HandleTypeDef* hspi_;       //用哪个spi通道？
    GPIO_TypeDef* cs_accel_port_;   //加速度计片选端口
    uint16_t cs_accel_pin_;         //加速度计片选引脚
    GPIO_TypeDef* cs_gyro_port_;    //陀螺仪片选端口
    uint16_t cs_gyro_pin_;          //陀螺仪片选引脚

    // ====== 数据 ======
    RawData raw_;
    RealData real_;

    float accel_sen_;               //加速度计灵敏度（量程）
    float gyro_sen_;                //陀螺仪灵敏度（量程）
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void BMI088_Read(float gyro[3], float accel[3], float *temperate);
    void BMI088_Init();
#ifdef __cplusplus
}
#endif


#endif //STARTM3508_BMI088_H