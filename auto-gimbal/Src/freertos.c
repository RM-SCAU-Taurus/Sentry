/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "comm_task.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "status_task.h"
#include "bsp_judge.h"
#include "test_task.h"
#include "vision_send_task.h"
#include "usb_task.h"
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
osThreadId mode_sw_task_t;
osThreadId chassis_task_t;
osThreadId gimbal_task_t;
osThreadId shoot_task_t;

osThreadId judge_send_task_t;
osThreadId usart_msg_send_task_t;
osThreadId can_msg_send_task_t;

osThreadId status_task_t;
osThreadId test_task_t;

osThreadId decode_task_handle;
osThreadId vision_send_task_handle;
osThreadId usb_task_handle;

 QueueHandle_t CDC_send_queue; // 新建CDC消息队列

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void decode_task(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
    taskENTER_CRITICAL();


  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */


/***************************High priority task***************************/
    osThreadDef(canTask, can_msg_send_task, osPriorityHigh, 0, 512);
    can_msg_send_task_t = osThreadCreate(osThread(canTask), NULL);
		
		osThreadDef(USBTask, usb_task, osPriorityHigh, 0, 128);
    usb_task_handle = osThreadCreate(osThread(USBTask), NULL);
	
    osThreadDef(DecodeTask, decode_task, osPriorityHigh, 0, 128);
    decode_task_handle = osThreadCreate(osThread(DecodeTask), NULL);
		    
		osThreadDef(Vision_send_task, vision_send_task, osPriorityHigh, 0, 128);
    vision_send_task_handle = osThreadCreate(osThread(Vision_send_task), NULL);
		
		osThreadDef(testTask, gimbal_to_chassic_task, osPriorityHigh, 0, 256);
    test_task_t = osThreadCreate(osThread(testTask),NULL);
		
		osThreadDef(statusTask, status_task, osPriorityHigh, 0, 128);
	status_task_t = osThreadCreate(osThread(statusTask),NULL);
    /***********************AboveNormal priority task***********************/
    osThreadDef(chassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
   chassis_task_t = osThreadCreate(osThread(chassisTask),NULL);

    osThreadDef(gimbalTask, gimbal_task, osPriorityAboveNormal, 0, 512);
   gimbal_task_t = osThreadCreate(osThread(gimbalTask),NULL);

    osThreadDef(shootTask, shoot_task, osPriorityAboveNormal, 0, 256);
    shoot_task_t = osThreadCreate(osThread(shootTask),NULL);

    /**************************Normal priority task**************************/
    osThreadDef(modeswTask, mode_switch_task, osPriorityNormal, 0, 128);
    mode_sw_task_t = osThreadCreate(osThread(modeswTask),NULL);

    osThreadDef(judgesendTask, judge_send_task, osPriorityNormal, 0, 128);
    judge_send_task_t = osThreadCreate(osThread(judgesendTask),NULL);
    

    /* low priority task */

		 CDC_send_queue = xQueueCreate(1, 128); // 串口消息队列
    taskEXIT_CRITICAL();
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
  /* init code for USB_DEVICE */
  //MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(100);
    }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
