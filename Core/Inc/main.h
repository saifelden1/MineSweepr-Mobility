#pragma once
#include "stm32f4xx_hal.h"
void Error_Handler(void);
void board_init(void);
extern UART_HandleTypeDef huart1, huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim3, htim4;
