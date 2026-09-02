#include "ads1232_service.h"
#include "ads1232.h"
#include "weight_processor.h"
#include <stdio.h>
#include <stddef.h>

void ADS1232_Service10Hz(UART_HandleTypeDef *huart, uint8_t output_enabled)
{
    static uint32_t next_tick;
    static uint8_t started;
    uint32_t raw24;
    int32_t value;
    WeightProcessor_Result result;
    char line[72];
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

    if (ADS1232_ReadRaw24(&raw24) == HAL_OK) {
        if ((raw24 & 0x00800000UL) != 0U) {
            value = (int32_t)(raw24 | 0xFF000000UL);
        } else {
            value = (int32_t)raw24;
        }
        /* 获取单片机上电后的运行时间，用于查找 Flash 中保存的动态零点。
         * 当前采样值可能已经包含负载，因此绝不使用当前值自动修正零点。 */
        (void)WeightProcessor_UpdateTimed(value, HAL_GetTick(), &result);
        if (output_enabled) {
            int64_t weight_magnitude;
            long weight_whole;
            long weight_fraction;

            weight_magnitude = result.weight_x10;
            if (weight_magnitude < 0) {
                weight_magnitude = -weight_magnitude;
                weight_whole = (long)(weight_magnitude / 10L);
                weight_fraction = (long)(weight_magnitude % 10L);
                length = snprintf(line, sizeof(line), "%lu,%ld,-%ld.%ld\n",
                                  (unsigned long)raw24,
                                  (long)result.filtered_raw,
                                  weight_whole, weight_fraction);
            } else {
                weight_whole = (long)(weight_magnitude / 10L);
                weight_fraction = (long)(weight_magnitude % 10L);
                length = snprintf(line, sizeof(line), "%lu,%ld,%ld.%ld\n",
                                  (unsigned long)raw24,
                                  (long)result.filtered_raw,
                                  weight_whole, weight_fraction);
            }
            if (length > 0) {
                (void)HAL_UART_Transmit(huart, (uint8_t *)line,
                                        (uint16_t)length, 20U);
            }
        }
    } else if (output_enabled) {
        static const uint8_t timeout_line[] = "ERR_TIMEOUT\n";
        (void)HAL_UART_Transmit(huart, (uint8_t *)timeout_line,
                                (uint16_t)(sizeof(timeout_line) - 1U), 20U);
    }

    /* 如果本次转换超过发送周期，则从当前时刻重新安排下一次采样。 */
    if ((int32_t)(HAL_GetTick() - next_tick) >= 0) {
        next_tick = HAL_GetTick() + 100U;
    }
}
