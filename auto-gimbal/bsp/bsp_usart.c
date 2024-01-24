/* HAL库 ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* C库 ---------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "stm32f4xx_hal.h"
#include "string.h"
/* USER CODE END */

/* 硬件外设库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 任务库 ------------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "cmsis_os.h"
#include "uart_decode.h"
#include "status_task.h"
#include "comm_task.h"
/* USER CODE END */

/* 数学库 ------------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 数据处理库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "remote_msg.h"
/* USER CODE END */

/* 类型定义库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 板级支持库 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#include "bsp_usart.h"
#include "bsp_judge.h"
/* USER CODE END */

/* 外部变量声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */
extern TaskHandle_t uart_decode_task_t;
extern SemaphoreHandle_t Decode_DBUS_Handle;  // 信号量句柄
extern SemaphoreHandle_t Decode_JUDGE_Handle; // 信号量句柄
/* USER CODE END */

/* 外部函数声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */

/* USER CODE END */

/* 静态函数声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */
static void UARTX_init(UART_HandleTypeDef *huart, uint16_t LEN);
static void Memory_change(UART_HandleTypeDef *huart, rx_msg_t *rx_msg, uint16_t LEN);
/* USER CODE END */

/* 宏定义声明 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#ifndef ABS
#define ABS(x) ((x > 0) ? (x) : (-(x)))
#endif
/* USER CODE END */

/* 结构体定义 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
DoubleBuffer_t DoubleBuffer_dbus;
DoubleBuffer_t DoubleBuffer_judge;

rx_msg_t rx_msg_dbus;
rx_msg_t rx_msg_judge;
/* USER CODE END */

/* 变量声明 ----------------------------------------------------------------------*/
/* USER CODE BEGIN */
static uint8_t dma_dbus_buf[2][DMA_DBUS_LEN];
static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];
/* USER CODE END */

/* 测试变量声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */
uint16_t Aerror_times= 0;
uint16_t Berror_times= 0;
volatile int uartDecodeSignal = 0;
/* USER CODE END */

/* 定义绝对值函数 */

void USER_UART_IDLECallback(UART_HandleTypeDef *huart, uint8_t *buf)
{
	if (huart->Instance == USART1) // DBUS
	{
			fifo_s_puts(&DBUS_fifo, (char *)buf, DMA_DBUS_LEN);

			osSignalSet(uart_decode_task_t, DBUS_MSG_PUT);	
	}

	else if (huart->Instance == USART2) // JUDGE
	{
			fifo_s_puts(&JUDGE_fifo, (char *)buf, DMA_JUDGE_LEN);

			osSignalSet(uart_decode_task_t, JUDGE_MSG_PUT);	
	}
}

void USER_UART_Init()
{
	/*********************** DBUS INIT ***************************/

	UARTX_init(&DBUS_HUART, DMA_DBUS_LEN);

	/*********************** JUDGE INIT ***************************/

	UARTX_init(&JUDGE_HUART, DMA_JUDGE_LEN);
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
			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18) // 接收成功18个字节长度
				{
					USER_UART_IDLECallback(huart, &dma_dbus_buf[0][0]); // Memory_1
				}
				Aerror_times++;
			}
			else
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18)							// 接收成功18个字节长度
				{														// 处理遥控器数据
					USER_UART_IDLECallback(huart, &dma_dbus_buf[1][0]); // Memory_1
				}
				Berror_times++;
			}
		}
		
		if (huart->Instance == USART2)
		{
			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				Memory_change(huart, &rx_msg_judge, DMA_JUDGE_LEN);
				if (rx_msg_dbus.rxlen_rx == 122) // 接收成功18个字节长度
				{
					USER_UART_IDLECallback(huart, &dma_judge_buf[0][0]); // Memory_1
				}
			}
			else
			{
				Memory_change(huart, &rx_msg_judge, DMA_JUDGE_LEN);
				if (rx_msg_dbus.rxlen_rx == 122)							// 接收成功18个字节长度
				{														// 处理遥控器数据
					USER_UART_IDLECallback(huart, &dma_judge_buf[1][0]); // Memory_1
				}
			}
		}
		
		
	}
}
static void UARTX_init(UART_HandleTypeDef *huart, uint16_t LEN)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t) & (huart->Instance->DR), (uint32_t)&dma_dbus_buf[0][0], (uint32_t)&dma_dbus_buf[1][0], LEN);
}

static void Memory_change(UART_HandleTypeDef *huart, rx_msg_t *rx_msg, uint16_t LEN)
{
	__HAL_DMA_DISABLE(huart->hdmarx);
	rx_msg->rxlen_rx = LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx); // 获取当前剩余数据量
	__HAL_DMA_SET_COUNTER(huart->hdmarx, LEN);					   // 重新设置数据量
	if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
		huart->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT); // 将DMA指向Memory1
	else
		huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT); // 将DMA指向Memory1
	__HAL_DMA_ENABLE(huart->hdmarx);
}
