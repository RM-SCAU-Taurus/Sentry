/**********C库****************/

/**********硬件外设库*********/

/**********任务库*************/
#include "cmsis_os.h"
#include "comm_task.h"
#include "uart_decode.h"
/**********数学库*************/

/**********数据处理库*********/
#include "remote_msg.h"
/**********类型定义库*********/

/**********板级支持库*********/

/**********外部变量声明*******/
extern SemaphoreHandle_t  Decode_DBUS_Handle;  // 信号量句柄
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  
/**********外部函数声明*******/

/**********静态函数声明*******/
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size);
/**********宏定义声明*********/

/**********结构体定义*********/

/**********变量声明***********/
fifo_s_t DBUS_fifo;// DBUS FIFO控制结构体
uint8_t DBUS_fifo_buf[3 * DMA_DBUS_LEN];// DBUS FIFO环形缓存区
uint8_t DBUS_de_buf[DMA_DBUS_LEN];// DBUS 缓存区

static uint8_t text_buf[18];				//发送缓冲区
/**********测试变量声明*******/

// FIFO初始化函数
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size)
{
  fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
  usb_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 3 * DMA_DBUS_LEN);
//	BaseType_t xReturn_JUDGE;
  for (;;)
  {
//			if(uartDecodeSignal == 1){
//      fifo_s_gets(&DBUS_fifo, (char *)DBUS_de_buf, DMA_DBUS_LEN);
//      rc_callback_handler(&rc, DBUS_de_buf);
//			uartDecodeSignal = 0;
//			}
		uint8_t len = fifo_s_used(&DBUS_fifo);
		if(len != 0 && HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY && uartDecodeSignal == 1 )
		{
			fifo_s_gets(&DBUS_fifo, (char *)text_buf, len);	//从 FIFO 取数据
			HAL_UART_Transmit_DMA(&huart3, text_buf, len);		//发送
			uartDecodeSignal = 0 ;
		}
			osDelayUntil(&mode_wake_time, 14);
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
			// 在F7系列是可以不写的，F1必须写
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_FLAG_TC4); //清除DMA2_Steam7传输完成标志
		HAL_UART_DMAStop(&huart3);		//传输完成以后关闭串口DMA,缺了这一句会死机


}
