#ifndef ADS1232_H
#define ADS1232_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef ADS1232_Init(void);
HAL_StatusTypeDef ADS1232_WaitReady(uint32_t timeout_ms);
HAL_StatusTypeDef ADS1232_ReadRaw24(uint32_t *raw24);
HAL_StatusTypeDef ADS1232_Read(int32_t *value);
void ADS1232_PowerDown(void);
void ADS1232_WakeUp(void);

#ifdef __cplusplus
}
#endif

#endif
