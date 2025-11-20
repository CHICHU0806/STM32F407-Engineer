#include "BMI088Middleware.h"
#include "main.h"
#include "BMI088reg.h"
#include "bsp_dwt.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_GPIO_init(void)
{
    // 如果使用 CubeMX 自动初始化，则不需要写
}

void BMI088_com_init(void)
{
    // SPI1 也是 CubeMX 初始化，不需要写
}

void BMI088_delay_ms(uint16_t ms)
{
    DWT_Delay_ms(ms);
}

void BMI088_delay_us(uint16_t us)
{
    DWT_Delay_us(us);
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

