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
uint8_t DBUS_fifo_buf[5 * DMA_DBUS_LEN];// DBUS FIFO环形缓存区
uint8_t DBUS_de_buf[2*DMA_DBUS_LEN];// DBUS 缓存区

/**********测试变量声明*******/

// FIFO初始化函数
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size)
{
  fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{
	uint32_t mode_wake_time = osKernelSysTick();
  usb_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 5 * DMA_DBUS_LEN);	
  for (;;)
  {
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      fifo_s_gets(&DBUS_fifo, (char *)DBUS_de_buf, 2*DMA_DBUS_LEN);
      rc_callback_handler(&rc, DBUS_de_buf);
			ulTaskNotifyTake(pdTRUE, 0);
//			osDelayUntil(&mode_wake_time, 7);
  }
}
