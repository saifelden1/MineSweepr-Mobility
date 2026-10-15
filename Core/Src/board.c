#include "main.h"
#include "mobility_config.h"
UART_HandleTypeDef huart1, huart2;
DMA_HandleTypeDef hdma_usart1_rx;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3, htim4;
static void clock_init(void)
{
    RCC_OscInitTypeDef osc={0}; RCC_ClkInitTypeDef clk={0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType=RCC_OSCILLATORTYPE_HSE; osc.HSEState=RCC_HSE_ON;
    osc.PLL.PLLState=RCC_PLL_ON; osc.PLL.PLLSource=RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM=25; osc.PLL.PLLN=192; osc.PLL.PLLP=RCC_PLLP_DIV2; osc.PLL.PLLQ=4;
    if (HAL_RCC_OscConfig(&osc)!=HAL_OK) Error_Handler();
    clk.ClockType=RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; clk.AHBCLKDivider=RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider=RCC_HCLK_DIV2; clk.APB2CLKDivider=RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk,FLASH_LATENCY_3)!=HAL_OK) Error_Handler();
}
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE(); __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}
void board_init(void)
{
    HAL_Init(); clock_init();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();

    GPIO_InitTypeDef pin = {0};

    /* Left Drivetrain Directions on GPIOB (FL: PB4, RL: PB5) */
    HAL_GPIO_WritePin(GPIOB, MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);
    pin.Pin = MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &pin);

    /* Right Drivetrain Directions on GPIOC (FR: PC13, RR: PC14) */
    HAL_GPIO_WritePin(GPIOC, MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);
    pin.Pin = MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &pin);

    /* Diagnostics & Indicators on GPIOB (PB12: Buzzer, PB13: LED1 Red, PB14: LED2 Yel, PB15: LED3 Gre) */
    HAL_GPIO_WritePin(GPIOB, MOBILITY_BUZZER_PIN | MOBILITY_LED1_PIN | MOBILITY_LED2_PIN | MOBILITY_LED3_PIN, GPIO_PIN_RESET);
    pin.Pin = MOBILITY_BUZZER_PIN | MOBILITY_LED1_PIN | MOBILITY_LED2_PIN | MOBILITY_LED3_PIN;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pin);

    /* 4-Channel Motor PWM on TIM4 (PB6: CH1, PB7: CH2, PB8: CH3, PB9: CH4) */
    pin.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    pin.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &pin);

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.Period = MOBILITY_PWM_PERIOD;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef pwm = {0};
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim4, &pwm, TIM_CHANNEL_4) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

    /* USART1 on PA9 (TX) / PA10 (RX) for RPi micro-ROS at 921600 baud */
    pin.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &pin);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 921600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    /* USART2 on PA2 (TX) / PA3 (RX) for NEO-6M GPS at 9600 baud */
    pin.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    pin.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &pin);

    huart2.Instance = USART2;
    huart2.Init = huart1.Init;
    huart2.Init.BaudRate = 9600;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    /* I2C2 on PB10 (SCL, AF4) and PB3 (SDA, AF9) for MPU-6050 Fast Mode (400 kHz) */
    pin.Pin = GPIO_PIN_10;
    pin.Mode = GPIO_MODE_AF_OD;
    pin.Pull = GPIO_PULLUP;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    pin.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &pin);

    pin.Pin = GPIO_PIN_3;
    pin.Alternate = GPIO_AF9_I2C2;
    HAL_GPIO_Init(GPIOB, &pin);

    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 400000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
    htim3.Instance=TIM3; htim3.Init.Prescaler=9599; htim3.Init.Period=99;
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim3)!=HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(TIM3_IRQn,1,0); HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* main starts the independent 100 Hz safety interrupt after drive initialization. */
}
