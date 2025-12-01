//
// Created by 20852 on 2025/11/29.
//
//
#include "BMI088.h"
#include "bsp_dwt.h"

extern SPI_HandleTypeDef hspi1;

// 最大支持一次 DMA 读取的字节数（按需调整，但不要超出 MCU DMA/内存限制）
#ifndef BMI088_DMA_MAX_LEN
#define BMI088_DMA_MAX_LEN 32
#endif

static volatile uint8_t bmi088_spi_dma_done = 0;
static volatile uint8_t bmi088_spi_dma_error = 0;
static uint8_t bmi088_dma_tx_buf[BMI088_DMA_MAX_LEN];

//构造函数
BMI088::BMI088(SPI_HandleTypeDef* hspi,
                 GPIO_TypeDef* cs_accel_port, uint16_t cs_accel_pin,
                 GPIO_TypeDef* cs_gyro_port, uint16_t cs_gyro_pin)
    : hspi_(hspi),
      cs_accel_port_(cs_accel_port), cs_accel_pin_(cs_accel_pin),
      cs_gyro_port_(cs_gyro_port), cs_gyro_pin_(cs_gyro_pin){}

//初始化BMI088
uint8_t BMI088::init() {
    uint8_t error = 0;

    //此处省略 GPIO 和 SPI 初始化，假设已经在外部完成(如果你要迁移到别的工程中)

    //通过下方运算可以通过返回的错误码判断问题原因（具体错误码看头文件enum中的内容）
    error |= accelInit();
    error |= gyroInit();

    return error;
}

//初始化加速度计
bool BMI088::accelInit() {
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //检查通信是否正常
    res = accelReadReg(BMI088_ACC_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = accelReadReg(BMI088_ACC_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //加速度计软件复位
    accelWriteReg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    DWT_Delay_ms(BMI088_LONG_DELAY_TIME);

    //复位后检查通信是否正常
    res = accelReadReg(BMI088_ACC_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = accelReadReg(BMI088_ACC_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查 "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // 设置加速度计配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        accelWriteReg(writeAccelConfig[write_reg_num][0],
                      writeAccelConfig[write_reg_num][1]);
        DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        res = accelReadReg(writeAccelConfig[write_reg_num][0]);
        DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != writeAccelConfig[write_reg_num][1]) {
            return writeAccelConfig[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

//初始化陀螺仪
bool BMI088::gyroInit() {
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //检查通信是否正常
    res = gyroReadReg(BMI088_GYRO_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = gyroReadReg(BMI088_GYRO_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //陀螺仪软件复位
    gyroWriteReg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    DWT_Delay_ms(BMI088_LONG_DELAY_TIME);

    //复位后检查通信是否正常
    res = gyroReadReg(BMI088_GYRO_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    res = gyroReadReg(BMI088_GYRO_CHIP_ID);
    DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查 "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // 设置陀螺仪配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        gyroWriteReg(writeGyroConfig[write_reg_num][0],
                     writeGyroConfig[write_reg_num][1]);
        DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        res = gyroReadReg(writeGyroConfig[write_reg_num][0]);
        DWT_Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        if (res != writeGyroConfig[write_reg_num][1]) {
            return writeGyroConfig[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

//读取加速度计,陀螺仪和温度计的原始数据
void BMI088::readRawData() {
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    //读取加速度计原始数据
    accelReadMulti(BMI088_ACCEL_XOUT_L, buf, 6);
    raw_.accel[0] = (int16_t)((buf[1] << 8) | buf[0]);
    raw_.accel[1] = (int16_t)((buf[3] << 8) | buf[2]);
    raw_.accel[2] = (int16_t)((buf[5] << 8) | buf[4]);

    //读取陀螺仪原始数据
    gyroReadMulti(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        raw_.gyro[0] = (int16_t)((buf[3] << 8) | buf[2]);
        raw_.gyro[1] = (int16_t)((buf[5] << 8) | buf[4]);
        raw_.gyro[2] = (int16_t)((buf[7] << 8) | buf[6]);
    }

    //读取温度原始数据
    accelReadMulti(BMI088_TEMP_M, buf, 2);
    raw_.temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    //处理温度补码
    if (raw_.temp > 1023)
    {
        raw_.temp -= 2048;
    }
}

//数据转换函数
void BMI088::convertData() {
    //加速度转换
    real_.accel[0] = raw_.accel[0] * BMI088_ACCEL_3G_SEN;
    real_.accel[1] = raw_.accel[1] * BMI088_ACCEL_3G_SEN;
    real_.accel[2] = raw_.accel[2] * BMI088_ACCEL_3G_SEN;
    //陀螺仪转换
    real_.gyro[0] = raw_.gyro[0] * BMI088_GYRO_2000_SEN;
    real_.gyro[1] = raw_.gyro[1] * BMI088_GYRO_2000_SEN;
    real_.gyro[2] = raw_.gyro[2] * BMI088_GYRO_2000_SEN;
    //温度转换
    real_.temp = (static_cast<float>(raw_.temp) * BMI088_TEMP_FACTOR) + BMI088_TEMP_OFFSET;
}

//读取数据函数
void BMI088::read(float gyro[3], float accel[3], float *temperate) {
    readRawData();
    convertData();
    accel[0] = real_.accel[0];
    accel[1] = real_.accel[1];
    accel[2] = real_.accel[2];
    gyro[0] = real_.gyro[0];
    gyro[1] = real_.gyro[1];
    gyro[2] = real_.gyro[2];
    *temperate = real_.temp;
}

//以下部分为工具函数，用于处理硬件通信
//拉低或者拉高片选引脚，用于信息的选择接收
void BMI088::csAccelLow() {
    HAL_GPIO_WritePin(cs_accel_port_, cs_accel_pin_, GPIO_PIN_RESET);
}

void BMI088::csAccelHigh() {
    HAL_GPIO_WritePin(cs_accel_port_, cs_accel_pin_, GPIO_PIN_SET);
}

void BMI088::csGyroLow() {
    HAL_GPIO_WritePin(cs_gyro_port_, cs_gyro_pin_, GPIO_PIN_RESET);
}

void BMI088::csGyroHigh() {
    HAL_GPIO_WritePin(cs_gyro_port_, cs_gyro_pin_, GPIO_PIN_SET);
}

//加速度计写单个寄存器
inline void BMI088::accelWriteReg(uint8_t reg, uint8_t data) {
    csAccelLow();
    BMI088_readandwrite_byte(reg);                           // 写操作，最高位为0
    BMI088_readandwrite_byte(data);
    csAccelHigh();
}

//加速度计读单个寄存器
inline uint8_t BMI088::accelReadReg(uint8_t reg) {
    csAccelLow();
    BMI088_readandwrite_byte(reg | 0x80);   // 发送寄存器地址 + 读标志
    BMI088_readandwrite_byte(0x55);         // dummy
    uint8_t data = BMI088_readandwrite_byte(0x55); // 接收数据
    csAccelHigh();
    return data;
}

//加速度计读多个寄存器
inline void BMI088::accelReadMulti(uint8_t reg, uint8_t *data, uint16_t len) {
    csAccelLow();
    BMI088_readandwrite_byte((reg) | 0x80);
    // 使用 DMA 读取（内部先发送 reg|0x80，再 DMA 收 len 字节）
    HAL_StatusTypeDef st = BMI088_read_multiple_reg_dma(reg, data, (uint8_t)len);
    // 如果 DMA 失败或超时，你可以选择 fallback 到原阻塞实现（可选）
    if (st != HAL_OK) {
        // fallback（可选）：按原逻辑用阻塞方式读取，保证可靠性
        BMI088_read_multiple_reg(reg, data, len); // 你原先的阻塞实现
    }             // 循环发送dummy以接收多次数据
    csAccelHigh();
}

//陀螺仪写单个寄存器
inline void BMI088::gyroWriteReg(uint8_t reg, uint8_t data) {
    csGyroLow();
    BMI088_readandwrite_byte(reg);                           // 写操作，最高位为0
    BMI088_readandwrite_byte(data);
    csGyroHigh();
}

//陀螺仪读单个寄存器
inline uint8_t BMI088::gyroReadReg(uint8_t reg) {
    csGyroLow();
    BMI088_readandwrite_byte(reg | 0x80);             // 读操作，最高位为1
    //HAL_SPI_Transmit(&hspi1, reinterpret_cast<const uint8_t *>(reg | 0x80), 1, 1000);
    uint8_t data = BMI088_readandwrite_byte(0x55);            // 发送一个dummy字节以接收数据
    csGyroHigh();
    return data;
}

//陀螺仪读多个寄存器
inline void BMI088::gyroReadMulti(uint8_t reg, uint8_t *data, uint16_t len) {
    csGyroLow();
    // 使用 DMA 读取（内部先发送 reg|0x80，再 DMA 收 len 字节）
    HAL_StatusTypeDef st = BMI088_read_multiple_reg_dma(reg, data, (uint8_t)len);
    // 如果 DMA 失败或超时，你可以选择 fallback 到原阻塞实现（可选）
    if (st != HAL_OK) {
        // fallback（可选）：按原逻辑用阻塞方式读取，保证可靠性
        BMI088_read_multiple_reg(reg, data, len); // 你原先的阻塞实现
    }           // 循环发送dummy以接收多次数据
    csGyroHigh();
}

//向IMU写+读单个字节
inline uint8_t BMI088::BMI088_readandwrite_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

// 从IMU读取多个寄存器数据
 void BMI088::BMI088_read_multiple_reg(uint8_t reg, uint8_t *buf, uint8_t len)
 {
     BMI088_readandwrite_byte(reg | 0x80);
     while (len != 0)
     {
         *buf = BMI088_readandwrite_byte(0x55);
         buf++;
         len--;
     }
 }

 // 返回 HAL_OK / HAL_ERROR / HAL_TIMEOUT
HAL_StatusTypeDef BMI088::BMI088_read_multiple_reg_dma(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (!buf || len == 0 || len > BMI088_DMA_MAX_LEN) return HAL_ERROR;

    bmi088_spi_dma_done = 0;
    bmi088_spi_dma_error = 0;

    // 1) 按原流程先发送寄存器地址 + 读标志。使用现有同步单字节函数保持时序一致性。
    //    这个调用会把寄存器地址发送出去（CS 必须已经拉低，调用者负责）。
    BMI088_readandwrite_byte(reg | 0x80);

    // 2) 填充 tx dummy buffer（0x55），长度为 len
    for (uint8_t i = 0; i < len; ++i) bmi088_dma_tx_buf[i] = 0x55;

    // 3) 启动 HAL SPI 双向 DMA 传输（发送 dummy，接收数据到 buf）
    //    这里使用 extern 定义的 hspi1（你的文件中已有 extern SPI_HandleTypeDef hspi1;）
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, bmi088_dma_tx_buf, buf, len) != HAL_OK) {
        return HAL_ERROR;
    }

    // 4) 等待 DMA 完成标志（短超时）——保持同步调用风格
    uint32_t t0 = HAL_GetTick();
    const uint32_t timeout_ms = 10; // 根据需要可调（不宜太大）
    while (!bmi088_spi_dma_done && !bmi088_spi_dma_error) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            // 超时：尝试中止 DMA，清状态
            HAL_SPI_Abort(&hspi1);
            return HAL_TIMEOUT;
        }
    }

    if (bmi088_spi_dma_error) {
        return HAL_ERROR;
    }

    return HAL_OK;
}


//加速度计配置状态检查数组写入
uint8_t BMI088::writeAccelConfig[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

//陀螺仪配置状态检查数组写入
uint8_t BMI088::writeGyroConfig[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};


//C接口函数
extern "C"{
    static BMI088 bmi088_instance(&hspi1,
                                  GPIOA, GPIO_PIN_4,
                                  GPIOB, GPIO_PIN_0);

    void BMI088_Init() {
        bmi088_instance.init();
    }

    void BMI088_Read(float gyro[3], float accel[3], float *temperate) {
        bmi088_instance.read(gyro, accel, temperate);
    }

    // SPI 传输完成（TxRx 完成）回调
    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
    {
        if (hspi == &hspi1) {
            bmi088_spi_dma_done = 1;
        }
    }

    // SPI 错误回调
    void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
    {
        if (hspi == &hspi1) {
            bmi088_spi_dma_error = 1;
        }
    }

}