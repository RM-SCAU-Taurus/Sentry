/**********C��****************/

/**********Ӳ�������*********/

/**********�����*************/
#include "cmsis_os.h"
#include "comm_task.h"
#include "uart_decode.h"
/**********��ѧ��*************/

/**********���ݴ����*********/
#include "remote_msg.h"
/**********���Ͷ����*********/

/**********�弶֧�ֿ�*********/

/**********�ⲿ��������*******/
extern SemaphoreHandle_t  Decode_DBUS_Handle;  // �ź������
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  
/**********�ⲿ��������*******/

/**********��̬��������*******/
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size);
/**********�궨������*********/

/**********�ṹ�嶨��*********/

/**********��������***********/
fifo_s_t DBUS_fifo;// DBUS FIFO���ƽṹ��
<<<<<<< HEAD
fifo_s_t JUDGE_fifo;
uint8_t DBUS_fifo_buf[3 * DMA_DBUS_LEN];// DBUS FIFO���λ�����
=======
uint8_t DBUS_fifo_buf[4 * DMA_DBUS_LEN];// DBUS FIFO���λ�����
>>>>>>> parent of 51913b3 (又改了一版dma)
uint8_t DBUS_de_buf[DMA_DBUS_LEN];// DBUS ������

uint8_t JUDGE_fifo_buf[3 * DMA_JUDGE_LEN];// JUDGE FIFO���λ�����
uint8_t JUDGE_de_buf[DMA_JUDGE_LEN];// JUDGE ������

static uint8_t text_buf[18];				//���ͻ�����
/**********���Ա�������*******/

// FIFO��ʼ������
static void usb_fifo_init(fifo_s_t *fifo_s, uint8_t *buf, uint16_t size)
{
  fifo_s_init(fifo_s, buf, size);
}

void uart_decode_task(void const *argu)
{
<<<<<<< HEAD
	uint32_t mode_wake_time = osKernelSysTick();
  usb_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 3 * DMA_DBUS_LEN);
	usb_fifo_init(&JUDGE_fifo, JUDGE_fifo_buf, 3 * DMA_JUDGE_LEN);
//	BaseType_t xReturn_JUDGE;
  for (;;)
  {
//			if(uartDecodeSignal == 1){
//      fifo_s_gets(&DBUS_fifo, (char *)DBUS_de_buf, DMA_DBUS_LEN);
//      rc_callback_handler(&rc, DBUS_de_buf);
//			uartDecodeSignal = 0;
//			}
		uint8_t len = fifo_s_used(&JUDGE_fifo);
		if(len != 0 && HAL_DMA_GetState(&hdma_usart3_tx) == HAL_DMA_STATE_READY)
		{
			fifo_s_gets(&JUDGE_fifo, (char *)text_buf, len);	//�� FIFO ȡ����
			HAL_UART_Transmit_DMA(&huart3, text_buf, len);			//����
			memset(text_buf,0,sizeof(text_buf));
			uartDecodeSignal = 0 ;
		}
			osDelayUntil(&mode_wake_time, 5);
=======

  usb_fifo_init(&DBUS_fifo, DBUS_fifo_buf, 4 * DMA_DBUS_LEN);
	BaseType_t xReturn_DBUS;
//	BaseType_t xReturn_JUDGE;
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
>>>>>>> parent of 51913b3 (又改了一版dma)
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
			// ��F7ϵ���ǿ��Բ�д�ģ�F1����д
//		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_tx, DMA_FLAG_TC4); //���DMA2_Steam7������ɱ�־
		HAL_UART_DMAStop(&huart3);		//��������Ժ�رմ���DMA,ȱ����һ�������


}
