
/**********C库*************************/
#include <stdio.h>
/**********硬件外设库******************/

/**********任务库**********************/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "sys_init.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "shoot_task.h"
#include "vision_predict.h"
#include "uart_decode.h"
#include "decode_camp.h"
/**********数学库**********************/

/**********硬件外设库******************/
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
/**********数据处理库******************/

/**********类型定义库******************/

/**********板级支持库******************/
#include "bsp_powerlimit.h"
#include "bsp_usart.h"
#include "bsp_can.h"
/**********外部变量声明****************/

/**********外部函数声明****************/

/**********静态函数声明****************/

/**********宏定义声明******************/

/**********结构体定义******************/

/**********变量声明********************/
fifo_s_t DBUS_fifo;						 // DBUS FIFO控制结构体
uint8_t DBUS_fifo_buf[3 * DMA_DBUS_LEN]; // DBUS FIFO环形缓存区

fifo_s_t JUDGE_fifo;						 // JUDGE FIFO控制结构体
uint8_t JUDGE_fifo_buf[2 * DMA_JUDGE_LEN]; // JUDGE FIFO环形缓存区
/**********测试变量声明****************/
//unsigned portBASE_TYPE uxHighWaterMark_sys_init;


void sys_init_task(void const *argu)
{
	taskENTER_CRITICAL();
	can_device_init();
	chassis_init();
	gimbal_param_init();
	PowerControl_Init();
	USER_UART_Init();
	vsn_init();
	MX_USB_DEVICE_Init();
	usb_fifo_init();
	UART_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 3 * DMA_DBUS_LEN);
	UART_fifo_init(&JUDGE_fifo, JUDGE_fifo_buf, 2 * DMA_JUDGE_LEN);

	HAL_Delay(1000);
	vTaskDelete(NULL);// 删除自身
	taskEXIT_CRITICAL();
//	uxHighWaterMark_sys_init = uxTaskGetStackHighWaterMark(NULL);
	
	
}

uint8_t InfoBuffer[1000];

void CPU_RunTime(void const *argu)
{
	
//	portTickType xLastWakeTime;
//	xLastWakeTime = xTaskGetTickCount();
//	HAL_TIM_Base_Start_IT(&htim3);//测试cpu占用
//	while(1)
//		{
//			
//		  osDelayUntil(&xLastWakeTime,1);//1000HZ
////		  vTaskList((char *)&InfoBuffer);
//		  vTaskGetRunTimeStats((char *)&InfoBuffer); 
//	   if(1)
//		   {
////				 printf("=================================================\r\n");
////				 printf("任务名        任务状态 优先级  剩余栈  任务号 \r\n");
////				 printf("=================================================\r\n");
////         printf("%s\r\n", InfoBuffer);
////				 printf("  B：阻塞  R：就绪  D：删除  S：暂停  X：运行 \r\n");
////				 printf("=================================================\r\n");
//				 
//				 printf("=================================================\r\n");
//			   printf("任务名         运行计数        CPU使用率 \r\n");
//				 printf("=================================================\r\n");
//		  	 printf("%s\r\n",InfoBuffer);
//				 printf("=================================================\r\n");
//				 printf("=================================================\r\n\n\n");
//				 vTaskDelay(1200);
//		   }//获取CPU使用权总
//    }	
}


int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);//发送字符
	
	return (ch);
}




