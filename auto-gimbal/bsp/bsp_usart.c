
#include "bsp_usart.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_judge.h"
#include "status_task.h"
#include "bsp_JY901.h"
#include "uart_decode.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "cmsis_os.h"

/* �������ֵ���� */
#ifndef ABS
    #define ABS(x)		((x>0)? (x): (-(x)))
#endif
extern TaskHandle_t uart_decode_task_t;
extern SemaphoreHandle_t  Decode_DBUS_Handle;  // �ź������
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  // �ź������
static void Memory_change(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer, DoubleBufferArrayPtr D_buf, uint16_t LEN);
static void UARTX_init(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer, uint8_t  * D_buf, uint16_t LEN);
volatile int uartDecodeSignal = 0;

int8_t rxlen_now;
int8_t rxlen_last;
int8_t rxlen_rx = 0;
static uint8_t dma_dbus_buf[2*DMA_DBUS_LEN];
static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_gyro_buf[DMA_GYRO_LEN];

DoubleBuffer_t DoubleBuffer_dbus;
DoubleBuffer_t DoubleBuffer_judge;

uint32_t enter=0;
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // DBUS
	{
		fifo_s_puts(&DBUS_fifo, (char*)dma_dbus_buf, 2*DMA_DBUS_LEN);
		memset(DoubleBuffer_dbus.last_buffer,0,2*DMA_DBUS_LEN);
		uartDecodeSignal = 1 ;
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

	UARTX_init(&DBUS_HUART, &DoubleBuffer_dbus,dma_dbus_buf, 2*DMA_DBUS_LEN);

	/*********************** JUDGE INIT ***************************/
	
}


void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart)
{
	if (RESET != __HAL_UART_GET_FLAG(huart,
									 UART_FLAG_IDLE))
	{
		 __HAL_UART_CLEAR_IDLEFLAG(huart);
//		 HAL_UART_AbortReceive(huart);
//		 __HAL_DMA_DISABLE(huart->hdmarx);
//		huart->RxState = HAL_UART_STATE_READY;
//		huart->hdmarx->State = HAL_DMA_STATE_READY; 
		 rxlen_now = huart->hdmarx->Instance->NDTR;
		 rxlen_rx = rxlen_last - rxlen_now;
		 rxlen_last =rxlen_now;
		/* re-use dma+idle to recv */
		if (huart->Instance == USART1)
		{
			if (ABS(rxlen_rx) == DMA_DBUS_LEN)
 			USER_UART_IDLECallback(huart); 
		}
		if (huart->Instance == USART2)
		{
			Memory_change(huart,&DoubleBuffer_judge,(DoubleBufferArrayPtr)dma_judge_buf,DMA_JUDGE_LEN);
			USER_UART_IDLECallback(huart); 
		}

		
	}
}
static void Memory_change(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer, DoubleBufferArrayPtr D_buf, uint16_t LEN)
{
	if (DoubleBuffer->current_buffer == &D_buf[Memory0][Memory0])
	{
		DoubleBuffer->current_buffer = &D_buf[Memory1][Memory0]; 
		DoubleBuffer->last_buffer = &D_buf[Memory0][Memory0];	  
		HAL_UART_Receive_DMA(huart, DoubleBuffer->current_buffer, LEN);
	}
	else if (DoubleBuffer->current_buffer == &D_buf[Memory1][Memory0])
	{
		DoubleBuffer->current_buffer = &D_buf[Memory0][Memory0];		
		DoubleBuffer->last_buffer = &D_buf[Memory1][Memory0];			
		HAL_UART_Receive_DMA(huart, DoubleBuffer->current_buffer, LEN); 
	}
	
}

static void UARTX_init(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer, uint8_t  * D_buf, uint16_t LEN)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	rxlen_now = 0;
	rxlen_last = 2*DMA_DBUS_LEN;
	HAL_UART_Receive_DMA(huart,D_buf, LEN);
}

