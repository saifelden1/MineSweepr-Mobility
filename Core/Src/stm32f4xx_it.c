#include "main.h"
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void PendSV_Handler(void) {}
void DebugMon_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void DMA2_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
