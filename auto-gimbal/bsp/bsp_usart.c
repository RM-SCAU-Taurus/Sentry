/* C�� ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "string.h"
/* USER CODE END */
/* HAL�� ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "stm32f4xx_hal_dma.h"
/* USER CODE END */

/* Ӳ������� --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ����� ------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "cmsis_os.h"
#include "uart_decode.h"
#include "status_task.h"
#include "comm_task.h"
/* USER CODE END */

/* ��ѧ�� ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ���ݴ���� --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "remote_msg.h"
/* USER CODE END */

/* ���Ͷ���� --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* �弶֧�ֿ� --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "bsp_usart.h"
#include "bsp_judge.h"
/* USER CODE END */

/* �ⲿ�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */
extern TaskHandle_t uart_decode_task_t;
extern SemaphoreHandle_t  Decode_DBUS_Handle;  // �ź������
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  // �ź������
/* USER CODE END */

/* �ⲿ�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* ��̬�������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */
static void UARTX_init(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer,uint16_t LEN,rx_msg_t *rx_msg);
/* USER CODE END */

/* �궨������ --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#ifndef ABS
    #define ABS(x)		((x>0)? (x): (-(x)))
#endif
/* USER CODE END */

/* �ṹ�嶨�� --------------------------------------------------------------------*/
/* USER CODE BEGIN */
DoubleBuffer_t DoubleBuffer_dbus;
DoubleBuffer_t DoubleBuffer_judge;

rx_msg_t rx_msg_dbus;
/* USER CODE END */

/* �������� ----------------------------------------------------------------------*/
/* USER CODE BEGIN */
static uint8_t dma_vision_buf[DMA_VISION_LEN];
static uint8_t dma_gyro_buf[DMA_GYRO_LEN];
static uint8_t dma_dbus_buf[2][DMA_DBUS_LEN];
//static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];
/* USER CODE END */

/* ���Ա������� ------------------------------------------------------------------*/
/* USER CODE BEGIN */
uint16_t error_times = 0 ;
volatile int uartDecodeSignal = 0;
/* USER CODE END */

/* �������ֵ���� */




void USER_UART_IDLECallback(UART_HandleTypeDef *huart,uint8_t *buf)
{
	if (huart->Instance == USART1) // DBUS
	{
		if(uart_decode_task_t != NULL){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;	
			fifo_s_puts(&DBUS_fifo, (char*)buf, DMA_DBUS_LEN);
			vTaskNotifyGiveFromISR(uart_decode_task_t,&xHigherPriorityTaskWoken);
			if(xHigherPriorityTaskWoken == pdTRUE)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    // ���֪ͨ�������������������л�
		}
	}

	else if (huart->Instance == USART2) // JUDGE
	{
//		judge_data_handler(DoubleBuffer_judge.last_buffer);
//		memset(DoubleBuffer_judge.last_buffer, 0, sizeof(DMA_JUDGE_LEN));
//		xSemaphoreGiveFromISR(Decode_JUDGE_Handle, &pxHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
	}
}


void USER_UART_Init()
{
	/*********************** DBUS INIT ***************************/

	UARTX_init(&DBUS_HUART, &DoubleBuffer_dbus,DMA_DBUS_LEN,&rx_msg_dbus);

	/*********************** JUDGE INIT ***************************/
	
}


void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart)
{
	if (RESET != __HAL_UART_GET_FLAG(huart,
									 UART_FLAG_IDLE))
	{
		 __HAL_UART_CLEAR_IDLEFLAG(huart);

		/* re-use dma+idle to recv */
		if (huart->Instance == USART1)
		{
			  if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET){
            __HAL_DMA_DISABLE(huart->hdmarx);
					  rx_msg_dbus.rxlen_rx = DMA_DBUS_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);	//��ȡ��ǰʣ��������
						__HAL_DMA_SET_COUNTER(huart->hdmarx,DMA_DBUS_LEN);	//��������������
				  	huart->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT);  //��DMAָ��Memory1
					  __HAL_DMA_ENABLE(huart->hdmarx);
            if(rx_msg_dbus.rxlen_rx == 18)	//���ճɹ�18���ֽڳ���
							{            
									USER_UART_IDLECallback(huart,&dma_dbus_buf[0][0]);	//Memory_1
							 }
						else
							error_times++;
							
				}
				else{
            __HAL_DMA_DISABLE(huart->hdmarx);
					  rx_msg_dbus.rxlen_rx = DMA_DBUS_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);	//��ȡ��ǰʣ��������
						__HAL_DMA_SET_COUNTER(huart->hdmarx,DMA_DBUS_LEN);	//��������������
						huart->hdmarx->Instance->CR  &= ~(DMA_SxCR_CT);  //��DMAָ��Memory1
					  __HAL_DMA_ENABLE(huart->hdmarx);
						if(rx_msg_dbus.rxlen_rx == 18)	//���ճɹ�18���ֽڳ���
							{                //����ң��������
							USER_UART_IDLECallback(huart,&dma_dbus_buf[1][0]);	//Memory_1
							}
							else
							error_times++;
				}

		}
	}
}
static void UARTX_init(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer,uint16_t LEN,rx_msg_t *rx_msg)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	rx_msg->rxlen_now = 0;
	rx_msg->rxlen_last = LEN;
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t)&(huart->Instance->DR),(uint32_t)&dma_dbus_buf[0][0],(uint32_t)&dma_dbus_buf[1][0],LEN);
	
//	HAL_UART_Receive_DMA(huart,&dma_dbus_buf[0][0], LEN);
	
}

