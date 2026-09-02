#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H
#include "usart.h"
#include <stdint.h>
void SerialDebug_Init(UART_HandleTypeDef *huart);
void SerialDebug_Process(void);
uint8_t SerialDebug_OutputEnabled(void);
void SerialDebug_RxCpltCallback(UART_HandleTypeDef *huart);
#endif
