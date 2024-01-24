/**********C库****************/

/**********硬件外设库*********/

/**********任务库*************/
#include "cmsis_os.h"
#include "comm_task.h"
#include "task.h"
#include "uart_decode.h"
#include "sys_init.h"
/**********数学库*************/

/**********数据处理库*********/
#include "remote_msg.h"
/**********类型定义库*********/

/**********板级支持库*********/
#include "bsp_judge.h"
/**********外部变量声明*******/
extern SemaphoreHandle_t Decode_DBUS_Handle; // 信号量句柄
extern SemaphoreHandle_t Decode_JUDGE_Handle;
/**********外部函数声明*******/

/**********静态函数声明*******/

/**********宏定义声明*********/

/**********结构体定义*********/

/**********变量声明***********/

uint8_t DBUS_de_buf[DMA_DBUS_LEN];		 // DBUS 缓存区

uint8_t JUDGE_de_buf[DMA_JUDGE_LEN];		 // JUDGE 缓存区


	
/**********测试变量声明*******/

// FIFO初始化函数
void UART_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size)
{
	fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{
	osEvent event;
	for (;;)
	{
		event = osSignalWait(DBUS_MSG_PUT | JUDGE_MSG_PUT, osWaitForever);
		if (event.status == osEventSignal)
		{
			if (event.value.signals & DBUS_MSG_PUT)
			{
				fifo_s_gets(&DBUS_fifo, (char *)DBUS_de_buf, DMA_DBUS_LEN);
				rc_callback_handler(&rc, DBUS_de_buf);
			}
			
			else if (event.value.signals & JUDGE_MSG_PUT)
			{
				fifo_s_gets(&JUDGE_fifo, (char *)JUDGE_de_buf, DMA_JUDGE_LEN);
				judge_data_handler(JUDGE_de_buf);
			}
			
		}
//			uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);//栈空间测量
	}
}
