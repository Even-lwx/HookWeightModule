#include "ads1232.h"
#include <stddef.h>

/* STM32G031C8T6 wiring: PB0=DOUT/DRDY, PB1=SCLK, PB2=PDWN. */
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

HAL_StatusTypeDef ADS1232_Read(int32_t *value)
{
    uint32_t raw = 0U;
    uint8_t i;

    if (value == NULL) {
        return HAL_ERROR;
    }
    if (ADS1232_WaitReady(120U) != HAL_OK) {
        return HAL_TIMEOUT;
    }

    /* ADS1232 data is a 24-bit MSB-first two's-complement word. */
    for (i = 0U; i < 24U; ++i) {
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_SET);
        __NOP();
        raw = (raw << 1) |
              ((HAL_GPIO_ReadPin(ADS1232_DOUT_PORT, ADS1232_DOUT_PIN) == GPIO_PIN_SET) ? 1U : 0U);
        HAL_GPIO_WritePin(ADS1232_SCLK_PORT, ADS1232_SCLK_PIN, GPIO_PIN_RESET);
        __NOP();
    }

    /* One extra clock forces DRDY/DOUT high. Two or more extra clocks
       would start offset calibration on ADS1232 and add about 800 ms
       settling time at 10 SPS. Channel and gain are hardware pins. */
    ADS1232_ClockPulse();

    if ((raw & 0x00800000UL) != 0U) {
        raw |= 0xFF000000UL;
    }
    *value = (int32_t)raw;
    return HAL_OK;
}
