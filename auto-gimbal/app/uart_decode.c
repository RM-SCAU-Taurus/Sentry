/**********C��****************/

/**********Ӳ�������*********/

/**********�����*************/
#include "cmsis_os.h"
#include "comm_task.h"
#include "task.h"
#include "uart_decode.h"
#include "sys_init.h"
/**********��ѧ��*************/

/**********���ݴ����*********/
#include "remote_msg.h"
/**********���Ͷ����*********/

/**********�弶֧�ֿ�*********/
#include "bsp_judge.h"
/**********�ⲿ��������*******/
extern SemaphoreHandle_t Decode_DBUS_Handle; // �ź������
extern SemaphoreHandle_t Decode_JUDGE_Handle;
/**********�ⲿ��������*******/

/**********��̬��������*******/

/**********�궨������*********/

/**********�ṹ�嶨��*********/

/**********��������***********/

uint8_t DBUS_de_buf[DMA_DBUS_LEN];		 // DBUS ������

uint8_t JUDGE_de_buf[DMA_JUDGE_LEN];		 // JUDGE ������


	
/**********���Ա�������*******/

// FIFO��ʼ������
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
//			uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);//ջ�ռ����
	}
}
