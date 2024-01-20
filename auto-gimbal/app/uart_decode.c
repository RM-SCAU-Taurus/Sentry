#include "uart_decode.h"

#include "comm_task.h"
#include "cmsis_os.h"
#include "remote_msg.h"

extern SemaphoreHandle_t  Decode_DBUS_Handle;  // 信号量句柄
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  
// 静态函数声明
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size);

// DBUS FIFO控制结构体
fifo_s_t DBUS_fifo;
// DBUS FIFO环形缓存区
uint8_t DBUS_fifo_buf[4 * DMA_DBUS_LEN];
// DBUS 缓存区
uint8_t DBUS_de_buf[DMA_DBUS_LEN];

// FIFO初始化函数
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size)
{
  fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{

  usb_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 4 * DMA_DBUS_LEN);
	BaseType_t xReturn_DBUS;
	BaseType_t xReturn_JUDGE;
  for (;;)
  {
		xReturn_DBUS  = xSemaphoreTake(Decode_DBUS_Handle,portMAX_DELAY);

    if(pdTRUE == xReturn_DBUS)
    {   
      fifo_s_gets(&DBUS_fifo, (char *)DBUS_de_buf, DMA_DBUS_LEN);
      rc_callback_handler(&rc, DBUS_de_buf);
      memset(DBUS_de_buf, 0, DMA_DBUS_LEN);
			uartDecodeSignal = Data_processing_completed;
    }//end of if(pdPASS = xReturn)
		
    osDelay(5);
  }
}
