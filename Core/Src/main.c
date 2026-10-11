/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32_Mobility ECU
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "mobility_pin_config.h"
#include "cytron_mdd10a_driver.h"
#include "mpu6050_driver.h"
#include "neo6m_driver.h"
#include "task_safety_watchdog.h"
#include "task_motor_control.h"
#include "task_sensor_acq.h"
#include "task_microros.h"
#include "microros_client.h"
#include "microros_transport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim4;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* FreeRTOS Thread Definitions */
osThreadId_t Task_SafetyWatchdogHandle;
const osThreadAttr_t Task_SafetyWatchdog_attributes = {
  .name = "Task_SafetyWDT",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t Task_MotorControlHandle;
const osThreadAttr_t Task_MotorControl_attributes = {
  .name = "Task_MotorCtrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

osThreadId_t Task_SensorAcqHandle;
const osThreadAttr_t Task_SensorAcq_attributes = {
  .name = "Task_SensorAcq",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t Task_MicroROSHandle;
const osThreadAttr_t Task_MicroROS_attributes = {
  .name = "Task_MicroROS",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize concrete BSP drivers */
  Cytron_MDD10A_Init(&htim4);
  MPU6050_Init(&hi2c2);
  NEO6M_Init(&huart2);

  /* Initialize micro-ROS transport and client */
  MicroROS_Transport_Init(&huart1, &hdma_usart1_rx, &hdma_usart1_tx);
  MicroROS_Client_Init();

  /* Initialize application tasks */
  Task_SafetyWatchdog_Init();
  Task_MotorControl_Init();
  Task_SensorAcq_Init();
  Task_MicroROS_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Create the thread(s) */
  Task_SafetyWatchdogHandle = osThreadNew(StartSafetyWatchdogTask, NULL, &Task_SafetyWatchdog_attributes);
  Task_MotorControlHandle   = osThreadNew(StartMotorControlTask, NULL, &Task_MotorControl_attributes);
  Task_SensorAcqHandle      = osThreadNew(StartSensorAcqTask, NULL, &Task_SensorAcq_attributes);
  Task_MicroROSHandle       = osThreadNew(StartMicroROSTask, NULL, &Task_MicroROS_attributes);

  /* Start scheduler */
  osKernelStart();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (25 MHz HSE -> 96 MHz SYSCLK)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Configure HSE 25 MHz oscillator with PLL multiplication to 96 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Fallback to HSI if external crystal fails */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  MOTOR_PWM_TIM_CLK_ENABLE();

  htim4.Instance = MOTOR_PWM_TIMER_INSTANCE;
  htim4.Init.Prescaler = MOTOR_PWM_PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MOTOR_PWM_PERIOD_TICKS;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&htim4);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_FL_PWM_CHANNEL);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_RL_PWM_CHANNEL);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_FR_PWM_CHANNEL);
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_RR_PWM_CHANNEL);
}

static void MX_I2C2_Init(void)
{
  IMU_I2C_CLK_ENABLE();

  hi2c2.Instance = IMU_I2C_INSTANCE;
  hi2c2.Init.ClockSpeed = IMU_I2C_SPEED_HZ;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
}

static void MX_USART1_UART_Init(void)
{
  MICROROS_UART_CLK_ENABLE();

  huart1.Instance = MICROROS_UART_INSTANCE;
  huart1.Init.BaudRate = MICROROS_BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

static void MX_USART2_UART_Init(void)
{
  GPS_UART_CLK_ENABLE();

  huart2.Instance = GPS_UART_INSTANCE;
  huart2.Init.BaudRate = GPS_UART_BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO Clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Cytron DIR Pins: PB4 (FL), PB5 (RL) */
  HAL_GPIO_WritePin(GPIOB, MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Cytron DIR Pins: PC13 (FR), PC14 (RR) */
  HAL_GPIO_WritePin(GPIOC, MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* TIM4 PWM Pins: PB6 (CH1), PB7 (CH2), PB8 (CH3), PB9 (CH4) */
  GPIO_InitStruct.Pin = MOTOR_FL_PWM_PIN | MOTOR_RL_PWM_PIN | MOTOR_FR_PWM_PIN | MOTOR_RR_PWM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* I2C2 Pins: PB10 (SCL), PB3 (SDA) */
  GPIO_InitStruct.Pin = IMU_I2C_SCL_PIN | IMU_I2C_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 Pins: PA9 (TX), PA10 (RX) */
  GPIO_InitStruct.Pin = MICROROS_UART_TX_PIN | MICROROS_UART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 Pins: PA2 (TX), PA3 (RX) */
  GPIO_InitStruct.Pin = GPS_UART_TX_PIN | GPS_UART_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
