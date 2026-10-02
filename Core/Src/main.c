/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for STM32_Mobility ECU.
  *                   Controls 4-motor differential skid-steer locomotion,
  *                   MPU-6050 IMU, NEO-6M GPS, and micro-ROS client.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Interfaces */
#include "imu_interface.h"
#include "motor_interface.h"
#include "gps_interface.h"

/* Concrete BSP Drivers */
#include "cytron_mdd10a_driver.h"
#include "mpu6050_driver.h"
#include "neo6m_driver.h"

/* Real-Time FreeRTOS Tasks */
#include "task_safety_watchdog.h"
#include "task_motor_control.h"
#include "task_sensor_acq.h"
#include "task_microros.h"

/* Peripheral Handles --------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* FreeRTOS Thread Definitions -----------------------------------------------*/
osThreadId_t Task_SafetyWatcHandle;
const osThreadAttr_t Task_SafetyWatc_attributes = {
  .name = "Task_SafetyWatc",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};

osThreadId_t Task_MotorContrHandle;
const osThreadAttr_t Task_MotorContr_attributes = {
  .name = "Task_MotorContr",
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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
#if (MOBILITY_PINOUT_SCHEME == 2)
static void MX_TIM3_Init(void);
#endif
static void MX_I2C_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock (96 MHz) */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
#if (MOBILITY_PINOUT_SCHEME == 2)
  MX_TIM3_Init();
#endif
  MX_I2C_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Initialize concrete BSP drivers */
  Cytron_MDD10A_Init();
  MPU6050_Init();
  NEO6M_Init();

  /* Init FreeRTOS scheduler */
  osKernelInitialize();

  /* Create the real-time threads */
  Task_SafetyWatcHandle = osThreadNew(StartSafetyWatchdogTask, NULL, &Task_SafetyWatc_attributes);
  Task_MotorContrHandle = osThreadNew(StartMotorControlTask, NULL, &Task_MotorContr_attributes);
  Task_SensorAcqHandle  = osThreadNew(StartSensorAcqTask, NULL, &Task_SensorAcq_attributes);
  Task_MicroROSHandle   = osThreadNew(StartMicroROSTask, NULL, &Task_MicroROS_attributes);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration (96 MHz SYSCLK from 16 MHz HSI / 25 MHz HSE)
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
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

/**
  * @brief TIM4 Initialization Function (20 kHz PWM on PB6..PB9, ARR=4799)
  */
static void MX_TIM4_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MOTOR_PWM_MAX_TICKS;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  /* Configure TIM4 Channel 1 (FL PWM - PB6) */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_FL_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure TIM4 Channel 2 (RL PWM - PB7) */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_RL_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

#if (MOBILITY_PINOUT_SCHEME != 2)
  /* Configure TIM4 Channel 3 (FR PWM - PB8) */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_FR_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure TIM4 Channel 4 (RR PWM - PB9) */
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, MOTOR_RR_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

#if (MOBILITY_PINOUT_SCHEME == 2)
static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = MOTOR_PWM_MAX_TICKS;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, MOTOR_FR_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, MOTOR_RR_PWM_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}
#endif

/**
  * @brief I2C Initialization Function (MPU-6050 @ 400 kHz Fast Mode)
  */
static void MX_I2C_Init(void)
{
#if (MOBILITY_PINOUT_SCHEME == 1)
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = IMU_I2C_SPEED_HZ;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
#else
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = IMU_I2C_SPEED_HZ;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
#endif
}

/**
  * @brief USART1 Initialization Function (micro-ROS transport @ 921600 baud)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = MICROROS_UART_INSTANCE;
  huart1.Init.BaudRate = MICROROS_UART_BAUDRATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function (NEO-6M GPS receiver @ 9600 baud)
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = GPS_UART_INSTANCE;
  huart2.Init.BaudRate = GPS_UART_BAUDRATE;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure FL DIR (PB4) and RL DIR (PB5) */
  HAL_GPIO_WritePin(GPIOB, MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_FL_DIR_PIN | MOTOR_RL_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure FR DIR (PC13) and RR DIR (PC14) */
  HAL_GPIO_WritePin(GPIOC, MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = MOTOR_FR_DIR_PIN | MOTOR_RR_DIR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  Period elapsed callback in non blocking mode (TIM11 SYS tick)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
