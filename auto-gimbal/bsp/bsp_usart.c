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
#include "stm32f4xx_hal_dma.h"
/* 定义绝对值函数 */
#ifndef ABS
    #define ABS(x)		((x>0)? (x): (-(x)))
#endif
extern TaskHandle_t uart_decode_task_t;
extern SemaphoreHandle_t  Decode_DBUS_Handle;  // 信号量句柄
extern SemaphoreHandle_t  Decode_JUDGE_Handle;  // 信号量句柄
static void Memory_change(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer, DoubleBufferArrayPtr D_buf, uint16_t LEN);
static void UARTX_init(UART_HandleTypeDef *huart, p_DoubleBuffer_t DoubleBuffer,uint16_t LEN,rx_msg_t *rx_msg);
static void RX_len_calcu(UART_HandleTypeDef *huart,rx_msg_t *rx_msg);

volatile int uartDecodeSignal = 0;

rx_msg_t rx_msg_dbus;
static uint8_t dma_dbus_buf[2][DMA_DBUS_LEN];
static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_gyro_buf[DMA_GYRO_LEN];

DoubleBuffer_t DoubleBuffer_dbus;
DoubleBuffer_t DoubleBuffer_judge;

uint32_t enter=0;
void USER_UART_IDLECallback(UART_HandleTypeDef *huart,uint8_t *buf)
{
	if (huart->Instance == USART1) // DBUS
	{
		if(uart_decode_task_t != NULL){
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;	
			fifo_s_puts(&DBUS_fifo, (char*)buf, DMA_DBUS_LEN);
			vTaskNotifyGiveFromISR(uart_decode_task_t,&xHigherPriorityTaskWoken);
			if(xHigherPriorityTaskWoken == pdTRUE)
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    // 如果通知了任务，则请求上下文切换
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
					  rx_msg_dbus.rxlen_rx = DMA_DBUS_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);	//获取当前剩余数据量
//						__HAL_DMA_SET_COUNTER(huart->hdmarx,18);	//重新设置数据量
					  __HAL_DMA_ENABLE(huart->hdmarx);
            if(rx_msg_dbus.rxlen_rx == 18)	//接收成功18个字节长度
							{            
									USER_UART_IDLECallback(huart,&dma_dbus_buf[0][0]);	//Memory_1
							 }
							
				}
				else{
            __HAL_DMA_DISABLE(huart->hdmarx);
					  rx_msg_dbus.rxlen_rx = DMA_DBUS_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);	//获取当前剩余数据量
//						__HAL_DMA_SET_COUNTER(huart->hdmarx,18);	//重新设置数据量
					  __HAL_DMA_ENABLE(huart->hdmarx);
						if(rx_msg_dbus.rxlen_rx == 18)	//接收成功18个字节长度
							{                //处理遥控器数据
									USER_UART_IDLECallback(huart,&dma_dbus_buf[1][0]);	//Memory_1
							}
				}

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

static void RX_len_calcu(UART_HandleTypeDef *huart,rx_msg_t *rx_msg){

	rx_msg->rxlen_now  = huart->hdmarx->Instance->NDTR;
	rx_msg->rxlen_rx   = rx_msg->rxlen_last - rx_msg->rxlen_now;
	rx_msg->rxlen_last = rx_msg->rxlen_now;
}


