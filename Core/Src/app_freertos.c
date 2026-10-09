/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
/* Definitions for SerialTxTask */
osThreadId_t SerialTxTaskHandle;
const osThreadAttr_t SerialTxTask_attributes = {
  .name = "SerialTxTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024 * 4
};
/* Definitions for MotorCtlTask */
osThreadId_t MotorCtlTaskHandle;
const osThreadAttr_t MotorCtlTask_attributes = {
  .name = "MotorCtlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for SerialCmdTask */
osThreadId_t SerialCmdTaskHandle;
const osThreadAttr_t SerialCmdTask_attributes = {
  .name = "SerialCmdTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 512 * 4
};
/* Definitions for NrfCmdTask */
osThreadId_t NrfCmdTaskHandle;
const osThreadAttr_t NrfCmdTask_attributes = {
  .name = "NrfCmdTask",
  .priority = (osPriority_t) osPriorityAboveNormal5,
  .stack_size = 1024 * 4
};
/* Definitions for VisionCmdTask */
osThreadId_t VisionCmdTaskHandle;
const osThreadAttr_t VisionCmdTask_attributes = {
  .name = "VisionCmdTask",
  .priority = (osPriority_t) osPriorityAboveNormal7,
  .stack_size = 512 * 4
};
/* Definitions for NrfIrqSem */
osSemaphoreId_t NrfIrqSemHandle;
const osSemaphoreAttr_t NrfIrqSem_attributes = {
  .name = "NrfIrqSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSerialTxTask(void *argument);
void StartMotorCtlTask(void *argument);
void StartSerialCmdTask(void *argument);
void StartNrfCmdTask(void *argument);
void StartVisionCmdTask(void *argument);

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

  /* Create the semaphores(s) */
  /* creation of NrfIrqSem */
  NrfIrqSemHandle = osSemaphoreNew(1, 1, &NrfIrqSem_attributes);

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
  /* creation of SerialTxTask */
  SerialTxTaskHandle = osThreadNew(StartSerialTxTask, NULL, &SerialTxTask_attributes);

  /* creation of MotorCtlTask */
  MotorCtlTaskHandle = osThreadNew(StartMotorCtlTask, NULL, &MotorCtlTask_attributes);

  /* creation of SerialCmdTask */
  SerialCmdTaskHandle = osThreadNew(StartSerialCmdTask, NULL, &SerialCmdTask_attributes);

  /* creation of NrfCmdTask */
  NrfCmdTaskHandle = osThreadNew(StartNrfCmdTask, NULL, &NrfCmdTask_attributes);

  /* creation of VisionCmdTask */
  VisionCmdTaskHandle = osThreadNew(StartVisionCmdTask, NULL, &VisionCmdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSerialTxTask */
/**
  * @brief  Function implementing the SerialTxTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSerialTxTask */
__weak void StartSerialTxTask(void *argument)
{
  /* USER CODE BEGIN StartSerialTxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSerialTxTask */
}

/* USER CODE BEGIN Header_StartMotorCtlTask */
/**
* @brief Function implementing the MotorCtlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorCtlTask */
__weak void StartMotorCtlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorCtlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartMotorCtlTask */
}

/* USER CODE BEGIN Header_StartSerialCmdTask */
/**
* @brief Function implementing the SerialCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialCmdTask */
__weak void StartSerialCmdTask(void *argument)
{
  /* USER CODE BEGIN StartSerialCmdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSerialCmdTask */
}

/* USER CODE BEGIN Header_StartNrfCmdTask */
/**
* @brief Function implementing the NrfCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNrfCmdTask */
__weak void StartNrfCmdTask(void *argument)
{
  /* USER CODE BEGIN StartNrfCmdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartNrfCmdTask */
}

/* USER CODE BEGIN Header_StartVisionCmdTask */
/**
* @brief Function implementing the VisionCmdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVisionCmdTask */
__weak void StartVisionCmdTask(void *argument)
{
  /* USER CODE BEGIN StartVisionCmdTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVisionCmdTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

