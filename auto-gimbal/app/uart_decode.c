#include "uart_decode.h"


#include "comm_task.h"
#include "cmsis_os.h"
#include "remote_msg.h"


// 静态函数声明
static void usb_fifo_init(fifo_s_t *fifo_s,uint8_t *buf,uint16_t size);

// DBUS FIFO控制结构体
fifo_s_t DBUS_fifo;
// DBUS FIFO环形缓存区
uint8_t DBUS_fifo_buf[5*DMA_DBUS_LEN];
// DBUS 缓存区
uint8_t DBUS_de_buf[DMA_DBUS_LEN];



// FIFO初始化函数
static void usb_fifo_init(fifo_s_t *fifo_s,uint8_t *buf,uint16_t size)
{
  fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{
	osEvent event;
	usb_fifo_init(&DBUS_fifo,DBUS_fifo_buf,5*DMA_DBUS_LEN);
  for (;;)
  {
    event = osSignalWait(UART_DECODE_DBUS_SEND | UART_DECODE_JUDGE_SEND ,osWaitForever);

    if (event.status == osEventSignal)
    {
			if(event.value.signals & UART_DECODE_DBUS_SEND)
			{
//				fifo_s_gets(&DBUS_fifo,(char *)DBUS_de_buf,DMA_DBUS_LEN);
//				rc_callback_handler(&rc,DBUS_de_buf);
//				memset(DBUS_de_buf, 0, DMA_DBUS_LEN);
			}
			
    }
  }
}
