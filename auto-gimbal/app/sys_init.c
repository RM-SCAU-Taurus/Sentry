
/**********C库*************************/

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

/**********测试变量声明****************/
unsigned portBASE_TYPE uxHighWaterMark_sys_init;


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
	HAL_Delay(1000);
//	uxHighWaterMark_sys_init = uxTaskGetStackHighWaterMark(NULL);
	vTaskDelete(NULL);
	
	taskEXIT_CRITICAL();
}
