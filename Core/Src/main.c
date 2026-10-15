#include "main.h"
#include "mobility_config.h"
#include "ros_interface.h"
#include "sensors.h"
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>
#include <string.h>
#include <time.h>

static uint8_t transport_rx[1024];
static size_t transport_tail;
static bool transport_open(struct uxrCustomTransport *transport)
{
    (void)transport; transport_tail=0;
    HAL_UART_AbortReceive(&huart1);
    return HAL_UART_Receive_DMA(&huart1,transport_rx,sizeof(transport_rx))==HAL_OK;
}
static bool transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    drive_stop(); return HAL_UART_AbortReceive(&huart1)==HAL_OK;
}
static size_t transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *error)
{
    (void)transport;
    if (len>65535 || HAL_UART_Transmit(&huart1,(uint8_t *)buf,(uint16_t)len,20)!=HAL_OK) {
        *error=1; return 0;
    }
    *error=0; return len;
}
static size_t transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *error)
{
    (void)transport;
    size_t got=0; uint32_t start=HAL_GetTick();
    do {
        size_t head=sizeof(transport_rx)-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        head%=sizeof(transport_rx);
        while (transport_tail!=head && got<len) {
            buf[got++]=transport_rx[transport_tail];
            transport_tail=(transport_tail+1U)%sizeof(transport_rx);
        }
        if (got || timeout<=0) break;
        HAL_Delay(1);
    } while (HAL_GetTick()-start<(uint32_t)timeout);
    *error=0; return got;
}
/* Bare-metal monotonic clock, with HAL's 32-bit rollover extended to 64 bits. */
int clock_gettime(clockid_t clock, struct timespec *time)
{
    (void)clock;
    static uint32_t previous;
    static uint64_t high;
    uint32_t tick=HAL_GetTick();
    if (tick<previous) high+=1ULL<<32;
    previous=tick;
    uint64_t ms=high+tick;
    time->tv_sec=(time_t)(ms/1000U); time->tv_nsec=(long)((ms%1000U)*1000000U);
    return 0;
}
int main(void)
{
    board_init(); drive_init();
    if (HAL_TIM_Base_Start_IT(&htim3)!=HAL_OK) Error_Handler();
    sensors_init();
    rmw_uros_set_custom_transport(true,NULL,transport_open,transport_close,transport_write,transport_read);
    bool connected=false;
    uint32_t ping_tick=HAL_GetTick();
    for (;;) {
        gps_poll();
        if (!connected) {
            drive_stop();
            if (HAL_GetTick()-ping_tick>=500U) {
                ping_tick=HAL_GetTick();
                if (rmw_uros_ping_agent(20,1)==RMW_RET_OK) connected=ros_entities_create();
            }
            HAL_Delay(1);
        } else {
            ros_spin();
            if (HAL_GetTick()-ping_tick>=500U) {
                ping_tick=HAL_GetTick();
                if (rmw_uros_ping_agent(20,1)!=RMW_RET_OK) {
                    ros_entities_destroy(); connected=false;
                }
            }
        }
        HAL_GPIO_WritePin(MOBILITY_LED3_PORT, MOBILITY_LED3_PIN, connected ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance==TIM3) motor_safety_tick();
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) { if (uart==&huart2) gps_rx_complete(); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart==&huart2) gps_uart_error();
    if (uart==&huart1) {
        drive_stop(); HAL_UART_AbortReceive(&huart1); transport_tail=0;
        HAL_UART_Receive_DMA(&huart1,transport_rx,sizeof(transport_rx));
    }
}
void Error_Handler(void)
{
    /* Direct register stop on all 4 channels works even before HAL handles have been initialized. */
    __HAL_RCC_TIM4_CLK_ENABLE();
    TIM4->CCR1=0; TIM4->CCR2=0; TIM4->CCR3=0; TIM4->CCR4=0;
    HAL_GPIO_WritePin(MOBILITY_LED1_PORT, MOBILITY_LED1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOBILITY_BUZZER_PORT, MOBILITY_BUZZER_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOBILITY_LED3_PORT, MOBILITY_LED3_PIN, GPIO_PIN_RESET);
    __disable_irq(); for (;;) {}
}
