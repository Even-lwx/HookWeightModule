#include "ads1232_service.h"
#include "ads1232.h"
#include <stdio.h>
#include <stddef.h>

void ADS1232_Service10Hz(UART_HandleTypeDef *huart)
{
    static uint32_t next_tick;
    static uint8_t started;
    int32_t value;
    char line[40];
    int length;

    if (huart == NULL) {
        return;
    }
    if (!started) {
        next_tick = HAL_GetTick();
        started = 1U;
    }
    if ((int32_t)(HAL_GetTick() - next_tick) < 0) {
        return;
    }
    next_tick += 100U;

    if (ADS1232_Read(&value) == HAL_OK) {
        /* FireWater sample frame: a single CSV value terminated by LF. */
        length = snprintf(line, sizeof(line), "%ld\n", (long)value);
        if (length > 0) {
            (void)HAL_UART_Transmit(huart, (uint8_t *)line,
                                    (uint16_t)length, 20U);
        }
    } else {
        static const uint8_t timeout_line[] = "ERR_TIMEOUT\n";
        (void)HAL_UART_Transmit(huart, (uint8_t *)timeout_line,
                                (uint16_t)(sizeof(timeout_line) - 1U), 20U);
    }

    /* If a conversion took longer than the period, restart from now. */
    if ((int32_t)(HAL_GetTick() - next_tick) >= 0) {
        next_tick = HAL_GetTick() + 100U;
    }
}
