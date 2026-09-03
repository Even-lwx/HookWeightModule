#include "ads1232.h"
#include <stddef.h>

/* STM32G031C8T6 接线：PB0=DOUT/DRDY，PB1=SCLK，PB2=PDWN。 */
#define ADS1232_DOUT_PORT GPIOB
#define ADS1232_DOUT_PIN  GPIO_PIN_0
#define ADS1232_SCLK_PORT GPIOB
#define ADS1232_SCLK_PIN  GPIO_PIN_1
#define ADS1232_PDWN_PORT GPIOB
#define ADS1232_PDWN_PIN  GPIO_PIN_2

static void ADS1232_ClockPulse(void)
{
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
    __NOP();
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
    __NOP();
}

void ADS1232_PowerDown(void)
{
    HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS1232_PDWN_PORT, ADS1232_PDWN_PIN, GPIO_PIN_RESET);
}

void ADS1232_WakeUp(void)
{
    HAL_GPIO_WritePin(ADS1232_PDWN_PORT, ADS1232_PDWN_PIN, GPIO_PIN_SET);
    HAL_Delay(2U);
}

HAL_StatusTypeDef ADS1232_Init(void)
{
    ADS1232_PowerDown();
    ADS1232_WakeUp();
    return ADS1232_WaitReady(200U);
}

HAL_StatusTypeDef ADS1232_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) != GPIO_PIN_RESET) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef ADS1232_ReadRaw24(uint32_t *raw24)
{
    uint32_t raw = 0U;
    uint8_t i;

    if (raw24 == NULL) {
        return HAL_ERROR;
    }
    if (ADS1232_WaitReady(120U) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    /* ADS1232 输出24位补码，最高位在前。 */
    for (i = 0U; i < 24U; ++i) {
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
        __NOP();
        raw = (raw << 1) |
              ((HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET) ? 1U : 0U);
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
        __NOP();
    }

    /* 读取24位数据后只追加1个时钟，使 DRDY/DOUT 恢复高电平。
       追加2个或更多时钟会启动芯片内部偏移校准，在10 SPS模式下会增加
       约800 ms的等待时间。通道和增益由芯片硬件引脚决定。 */
    ADS1232_ClockPulse();

    /* 原样返回 ADS1232 的 D23～D0，不进行符号扩展、缩放、零点修正或滤波。 */
    *raw24 = raw & 0x00FFFFFFUL;
    return HAL_OK;
}

HAL_StatusTypeDef ADS1232_Read(int32_t *value)
{
    uint32_t raw24;
    HAL_StatusTypeDef status;

    if (value == NULL) {
        return HAL_ERROR;
    }

    status = ADS1232_ReadRaw24(&raw24);
    if (status != HAL_OK) {
        return status;
    }

    if ((raw24 & 0x00800000UL) != 0U) {
        raw24 |= 0xFF000000UL;
    }
    *value = (int32_t)raw24;
    return HAL_OK;
}
