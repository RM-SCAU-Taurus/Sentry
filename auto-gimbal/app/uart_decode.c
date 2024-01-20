#include "uart_decode.h"


#include "comm_task.h"
#include "cmsis_os.h"
#include "remote_msg.h"


// ��̬��������
static void usb_fifo_init(fifo_s_t *fifo_s,uint8_t *buf,uint16_t size);

// DBUS FIFO���ƽṹ��
fifo_s_t DBUS_fifo;
// DBUS FIFO���λ�����
uint8_t DBUS_fifo_buf[5*DMA_DBUS_LEN];
// DBUS ������
uint8_t DBUS_de_buf[DMA_DBUS_LEN];



// FIFO��ʼ������
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
