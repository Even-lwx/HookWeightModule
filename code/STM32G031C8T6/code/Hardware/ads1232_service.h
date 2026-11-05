#ifndef ADS1232_SERVICE_H
#define ADS1232_SERVICE_H

#include "main.h"
#include "usart.h"

/* 在 ADS1232_Init() 后，每次主循环迭代调用一次。 */
void ADS1232_Service10Hz(UART_HandleTypeDef *huart, uint8_t output_enabled);

#endif /* ADS1232_SERVICE_H */
