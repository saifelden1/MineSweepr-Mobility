/**
 * @file microros_transport.c
 * @brief Configurable micro-ROS UART DMA Circular Transport implementation for STM32_Mobility.
 *
 * Implements non-blocking 2048-byte circular RX buffer with USART IDLE detection
 * and DMA normal TX with FreeRTOS CMSIS_V2 semaphore synchronization.
 */

#include "microros_transport.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static UART_HandleTypeDef *s_huart = &huart1;
static DMA_HandleTypeDef  *s_hdma_rx = &hdma_usart1_rx;
static DMA_HandleTypeDef  *s_hdma_tx = &hdma_usart1_tx;

static uint8_t s_rx_dma_buffer[MICROROS_RX_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx_dma_buffer[MICROROS_TX_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static volatile size_t s_rx_tail = 0;
static volatile bool s_tx_busy = false;
static bool s_is_open = false;

/* OS Synchronization Primitives */
static osSemaphoreId_t s_tx_semaphore = NULL;
static const osSemaphoreAttr_t s_tx_semaphore_attr = {
    .name = "MicroROS_Mob_TxSem"
};

static osMutexId_t s_rx_mutex = NULL;
static const osMutexAttr_t s_rx_mutex_attr = {
    .name = "MicroROS_Mob_RxMutex"
};

static osMutexId_t s_tx_mutex = NULL;
static const osMutexAttr_t s_tx_mutex_attr = {
    .name = "MicroROS_Mob_TxMutex"
};

bool (MicroROS_Transport_Init)(void)
{
    if (s_huart == NULL)
    {
        return false;
    }

    s_rx_tail = 0;
    s_tx_busy = false;

    memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));
    memset(s_tx_dma_buffer, 0, sizeof(s_tx_dma_buffer));

    if (s_tx_semaphore == NULL)
    {
        s_tx_semaphore = osSemaphoreNew(1, 0, &s_tx_semaphore_attr);
    }
    if (s_rx_mutex == NULL)
    {
        s_rx_mutex = osMutexNew(&s_rx_mutex_attr);
    }
    if (s_tx_mutex == NULL)
    {
        s_tx_mutex = osMutexNew(&s_tx_mutex_attr);
    }

    /* Configure and link TX DMA if not already linked by MSP */
    if (s_huart->hdmatx == NULL && s_hdma_tx != NULL)
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
        s_hdma_tx->Instance = DMA2_Stream7;
        s_hdma_tx->Init.Channel = DMA_CHANNEL_4;
        s_hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
        s_hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
        s_hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
        s_hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        s_hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        s_hdma_tx->Init.Mode = DMA_NORMAL;
        s_hdma_tx->Init.Priority = DMA_PRIORITY_MEDIUM;
        s_hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(s_hdma_tx);
        __HAL_LINKDMA(s_huart, hdmatx, *s_hdma_tx);

        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    }

    return MicroROS_Transport_Open();
}

bool MicroROS_Transport_InitHandles(void *huart, void *hdma_rx, void *hdma_tx)
{
    if (huart != NULL)   s_huart = (UART_HandleTypeDef*)huart;
    if (hdma_rx != NULL) s_hdma_rx = (DMA_HandleTypeDef*)hdma_rx;
    if (hdma_tx != NULL) s_hdma_tx = (DMA_HandleTypeDef*)hdma_tx;
    return (MicroROS_Transport_Init)();
}

bool MicroROS_Transport_Open(void)
{
    if (s_is_open)
    {
        return true;
    }
    if (s_huart == NULL)
    {
        return false;
    }

    s_rx_tail = 0;
    s_tx_busy = false;

    if (HAL_UART_Receive_DMA(s_huart, s_rx_dma_buffer, MICROROS_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return false;
    }

    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
    s_is_open = true;
    return true;
}

bool MicroROS_Transport_Close(void)
{
    if (!s_is_open || s_huart == NULL)
    {
        return false;
    }

    __HAL_UART_DISABLE_IT(s_huart, UART_IT_IDLE);
    HAL_UART_DMAStop(s_huart);
    s_is_open = false;
    return true;
}

bool MicroROS_Transport_IsOpen(void)
{
    return s_is_open;
}

size_t MicroROS_Transport_Available(void)
{
    if (!s_is_open || s_huart == NULL || s_huart->hdmarx == NULL)
    {
        return 0;
    }

    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(s_huart->hdmarx);
    size_t head = (MICROROS_RX_DMA_BUFFER_SIZE - dma_remaining) % MICROROS_RX_DMA_BUFFER_SIZE;

    if (head >= s_rx_tail)
    {
        return (head - s_rx_tail);
    }
    return (MICROROS_RX_DMA_BUFFER_SIZE - s_rx_tail + head);
}

size_t MicroROS_Transport_Read(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    if (!s_is_open || buf == NULL || max_len == 0)
    {
        return 0;
    }

    if (s_rx_mutex != NULL && osMutexAcquire(s_rx_mutex, pdMS_TO_TICKS(timeout_ms)) != osOK)
    {
        return 0;
    }

    size_t available = MicroROS_Transport_Available();
    size_t bytes_to_read = (available < max_len) ? available : max_len;

    for (size_t i = 0; i < bytes_to_read; i++)
    {
        buf[i] = s_rx_dma_buffer[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % MICROROS_RX_DMA_BUFFER_SIZE;
    }

    if (s_rx_mutex != NULL)
    {
        osMutexRelease(s_rx_mutex);
    }

    return bytes_to_read;
}

size_t MicroROS_Transport_Write(const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (!s_is_open || buf == NULL || len == 0 || s_huart == NULL)
    {
        return 0;
    }

    if (s_tx_mutex != NULL && osMutexAcquire(s_tx_mutex, pdMS_TO_TICKS(timeout_ms)) != osOK)
    {
        return 0;
    }

    size_t total_sent = 0;
    while (total_sent < len)
    {
        size_t chunk = (len - total_sent > MICROROS_TX_DMA_BUFFER_SIZE) ?
                        MICROROS_TX_DMA_BUFFER_SIZE : (len - total_sent);

        memcpy(s_tx_dma_buffer, &buf[total_sent], chunk);

        s_tx_busy = true;
        if (HAL_UART_Transmit_DMA(s_huart, s_tx_dma_buffer, (uint16_t)chunk) != HAL_OK)
        {
            s_tx_busy = false;
            break;
        }

        if (s_tx_semaphore != NULL)
        {
            if (osSemaphoreAcquire(s_tx_semaphore, pdMS_TO_TICKS(timeout_ms)) != osOK)
            {
                HAL_UART_AbortTransmit(s_huart);
                s_tx_busy = false;
                break;
            }
        }
        else
        {
            uint32_t start_tick = osKernelGetTickCount();
            while (s_tx_busy && (osKernelGetTickCount() - start_tick < pdMS_TO_TICKS(timeout_ms)))
            {
                osDelay(1);
            }
            if (s_tx_busy)
            {
                HAL_UART_AbortTransmit(s_huart);
                s_tx_busy = false;
                break;
            }
        }

        total_sent += chunk;
    }

    if (s_tx_mutex != NULL)
    {
        osMutexRelease(s_tx_mutex);
    }

    return total_sent;
}

bool MicroROS_Transport_Transmit(const uint8_t *data, uint16_t length)
{
    return (MicroROS_Transport_Write(data, length, MICROROS_TX_TIMEOUT_MS) == length);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart)
    {
        s_tx_busy = false;
        if (s_tx_semaphore != NULL)
        {
            osSemaphoreRelease(s_tx_semaphore);
        }
    }
}
