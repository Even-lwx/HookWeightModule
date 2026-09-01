#ifndef ADS1232_SERVICE_H
#define ADS1232_SERVICE_H

#include "main.h"
#include "usart.h"

/* Call once per main-loop iteration after ADS1232_Init(). */
void ADS1232_Service10Hz(UART_HandleTypeDef *huart);

#endif /* ADS1232_SERVICE_H */
