/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "Chassis_App.h"
#include "Gimbal_App.h"
#include "Shoot_App.h"
#include "Control_App.h"
#include "Timer_App.h"
#include "Comm_App.h"
#include "IMU_App.h"
#include "WatchDog_App.h"
#include "PC_UART_App.h"
#include "Referee_App.h"
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
QueueHandle_t UC_Pack_Queue;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
BoardStatus_t get_board_status();
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  osThreadId ChassisTaskHandle;
  osThreadId GimbalTaskHandle;
  osThreadId ShootTaskHandle;
  osThreadId TimerTaskHandle;
  osThreadId CommTaskHandle;
  osThreadId IMUTaskHandle;
  osThreadId RCTaskHandle;
  osThreadId WDGTaskHandle;
  osThreadId PCUARTTaskHandle;
  osThreadId RefTaskHandle;

  uint8_t board_status = get_board_status();
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
  UC_Pack_Queue = xQueueCreate(3, UC_PACK_SIZE);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(TimerTask, Timer_Task_Func, osPriorityHigh, 0, 256);
    TimerTaskHandle = osThreadCreate(osThread(TimerTask), (void*) &board_status);

    osThreadDef(CommTask, Comm_Task_Func, osPriorityHigh, 0, 256);
    CommTaskHandle = osThreadCreate(osThread(CommTask), (void*) &board_status);

//    osThreadDef(WDGTask, WatchDog_Task_Function, osPriorityHigh, 0, 256);
//    WDGTaskHandle = osThreadCreate(osThread(WDGTask), (void*) board_status);


    if (board_status == CHASSIS_BOARD) {
    	  osThreadDef(ChassisTask, Chassis_Task_Func, osPriorityRealtime, 0, 256);
    	  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    	  osThreadDef(RCTask, RC_Task_Func, osPriorityHigh, 0, 384);
    	  RCTaskHandle = osThreadCreate(osThread(RCTask), NULL);

//    	  osThreadDef(RefTask, Referee_Task_Func, osPriorityHigh, 0, 384);
//    	  RefTaskHandle = osThreadCreate(osThread(RefTask), NULL);

    } else if (board_status == GIMBAL_BOARD) {
    	  osThreadDef(GimbalTask, Gimbal_Task_Function, osPriorityRealtime, 0, 512);
    	  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

    	  osThreadDef(ShootTask, Shoot_Task_Func, osPriorityHigh, 0, 256);
    	  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

    	  osThreadDef(IMUTask, IMU_Task_Function, osPriorityHigh, 0, 256);
    	  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

//    	  osThreadDef(PCUARTTask, PC_UART_Func, osPriorityHigh, 0, 256);
//    	  PCUARTTaskHandle = osThreadCreate(osThread(PCUARTTask), NULL);
      }
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */
