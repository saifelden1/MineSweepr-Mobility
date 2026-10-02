/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines for STM32_Mobility.
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"
#include "neo6m_driver.h"
#include "microros_transport.h"

extern TIM_HandleTypeDef htim11;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim11);
}

/**
  * @brief This function handles USART2 global interrupt with IDLE detection.
  */
void USART2_IRQHandler(void)
{
  NEO6M_UART_IdleCallback();
  HAL_UART_IRQHandler(&huart2);
}

/**
  * @brief This function handles DMA1 Stream 5 global interrupt (USART2 RX).
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief This function handles USART1 global interrupt with IDLE detection.
  */
void USART1_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    MicroROS_Transport_OnUartIdle();
  }
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles DMA2 Stream 2 global interrupt (USART1 RX).
  */
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief This function handles DMA2 Stream 7 global interrupt (USART1 TX).
  */
void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
  * @brief Tx Transfer completed callback.
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    MicroROS_Transport_OnTxCplt();
  }
}

/**
  * @brief Rx Transfer completed callback.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    MicroROS_Transport_OnRxCplt();
  }
}

/**
  * @brief UART error callback.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    MicroROS_Transport_OnError();
  }
}

