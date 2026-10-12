/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_SafetyWDT */
osThreadId_t Task_SafetyWDTHandle;
const osThreadAttr_t Task_SafetyWDT_attributes = {
  .name = "Task_SafetyWDT",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task_MotorCtrl */
osThreadId_t Task_MotorCtrlHandle;
const osThreadAttr_t Task_MotorCtrl_attributes = {
  .name = "Task_MotorCtrl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_SensorAcq */
osThreadId_t Task_SensorAcqHandle;
const osThreadAttr_t Task_SensorAcq_attributes = {
  .name = "Task_SensorAcq",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_MicroROS */
osThreadId_t Task_MicroROSHandle;
const osThreadAttr_t Task_MicroROS_attributes = {
  .name = "Task_MicroROS",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSafetyWatchdogTask(void *argument);
void StartMotorControlTask(void *argument);
void StartSensorAcqTask(void *argument);
void StartMicroROSTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_SafetyWDT */
  Task_SafetyWDTHandle = osThreadNew(StartSafetyWatchdogTask, NULL, &Task_SafetyWDT_attributes);

  /* creation of Task_MotorCtrl */
  Task_MotorCtrlHandle = osThreadNew(StartMotorControlTask, NULL, &Task_MotorCtrl_attributes);

  /* creation of Task_SensorAcq */
  Task_SensorAcqHandle = osThreadNew(StartSensorAcqTask, NULL, &Task_SensorAcq_attributes);

  /* creation of Task_MicroROS */
  Task_MicroROSHandle = osThreadNew(StartMicroROSTask, NULL, &Task_MicroROS_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSafetyWatchdogTask */
/**
  * @brief  Function implementing the Task_SafetyWDT thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSafetyWatchdogTask */
__weak void StartSafetyWatchdogTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyWatchdogTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSafetyWatchdogTask */
}

/* USER CODE BEGIN Header_StartMotorControlTask */
/**
* @brief Function implementing the Task_MotorCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorControlTask */
__weak void StartMotorControlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorControlTask */
}

/* USER CODE BEGIN Header_StartSensorAcqTask */
/**
* @brief Function implementing the Task_SensorAcq thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorAcqTask */
__weak void StartSensorAcqTask(void *argument)
{
  /* USER CODE BEGIN StartSensorAcqTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensorAcqTask */
}

/* USER CODE BEGIN Header_StartMicroROSTask */
/**
* @brief Function implementing the Task_MicroROS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMicroROSTask */
__weak void StartMicroROSTask(void *argument)
{
  /* USER CODE BEGIN StartMicroROSTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMicroROSTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

