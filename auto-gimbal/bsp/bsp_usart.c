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
static void USART_Init_Configc_Create(USART_Init_Config_s *conf, uint8_t size, UART_HandleTypeDef *usart_handle, usart_module_callback callback,
									  uint8_t *Buf_0_addr, uint8_t *Buf_1_addr, uint8_t Frame_len);

static uint8_t *Memory_change(USARTInstance *_instance);
/* USER CODE END */

/* 宏定义声明 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
#ifndef ABS
#define ABS(x) ((x > 0) ? (x) : (-(x)))
#endif
/* USER CODE END */

/* 结构体定义 --------------------------------------------------------------------*/
/* USER CODE BEGIN */
USART_Init_Config_s USART_DBUS_Conf;
USART_Init_Config_s USART_JUDGE_Conf;

/* USER CODE END */

/* 变量声明 ----------------------------------------------------------------------*/
/* USER CODE BEGIN */
static uint8_t dma_dbus_buf[2][DMA_DBUS_LEN];
static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];

static uint8_t idx = 0;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/* USER CODE END */

/* 测试变量声明 ------------------------------------------------------------------*/
/* USER CODE BEGIN */

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

void USER_UART_Init(void)
{
	/*********************** DBUS INIT ***************************/
	USART_Init_Configc_Create(&USART_DBUS_Conf, DMA_DBUS_LEN, &huart1, USER_UART_IDLECallback, dma_dbus_buf[0], dma_dbus_buf[1], Frame_DBUS_LEN);
	USARTRegister(&USART_DBUS_Conf);

	/*********************** JUDGE INIT ***************************/

	USART_Init_Configc_Create(&USART_JUDGE_Conf, DMA_JUDGE_LEN, &huart2, USER_UART_IDLECallback, dma_judge_buf[0], dma_judge_buf[1], DMA_JUDGE_LEN);
	USARTRegister(&USART_JUDGE_Conf);
}

void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart)
{
	if (RESET != __HAL_UART_GET_FLAG(huart,
									 UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		for (uint8_t i = 0; i < idx; ++i)
		{
			if (huart == usart_instance[i]->usart_handle)
			{ // call the callback function if it is not NULL
				if (usart_instance[i]->module_callback != NULL)
				{
					usart_instance[i]->module_callback(usart_instance[i]->usart_handle, Memory_change(usart_instance[i]));//Memory_change返回上一帧使用的缓冲区 
					break;
				}
			}
		
		}
	}
}

static uint8_t *Memory_change(USARTInstance *_instance)
{
	__HAL_DMA_DISABLE(_instance->usart_handle->hdmarx);
	_instance->recv_buff_len = _instance->recv_buff_size - __HAL_DMA_GET_COUNTER(_instance->usart_handle->hdmarx); // 获取当前剩余数据量
	__HAL_DMA_SET_COUNTER(_instance->usart_handle->hdmarx, _instance->recv_buff_size);							   // 重新设置数据量
	if ((_instance->usart_handle->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
	{
		_instance->usart_handle->hdmarx->Instance->CR |= (uint32_t)(DMA_SxCR_CT); // 将DMA指向Memory1
		__HAL_DMA_ENABLE(_instance->usart_handle->hdmarx);
		return _instance->Buf_0;
	}
	else
	{
		_instance->usart_handle->hdmarx->Instance->CR &= ~(DMA_SxCR_CT); // 将DMA指向Memory1
		__HAL_DMA_ENABLE(_instance->usart_handle->hdmarx);
		return _instance->Buf_1;
	}
}

/**
 * @brief 注册串口实例
 *
 * @todo
 *
 *
 * @param
 */

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{

	USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
	memset(instance, 0, sizeof(USARTInstance));

	instance->Buf_0 = init_config->Buf_0;
	instance->Buf_1 = init_config->Buf_1;
	instance->recv_Frame_len = init_config->recv_Frame_len;
	instance->recv_buff_len = init_config->recv_buff_len;
	instance->usart_handle = init_config->usart_handle;
	instance->recv_buff_size = init_config->recv_buff_size;
	instance->module_callback = init_config->module_callback;

	usart_instance[idx++] = instance;
	USARTServiceInit(instance);
	return instance;
}

/**
 * @brief 启动串口DMA接收,会在每个实例注册之后自动启用接收,当前实现为DMA接收,后续可能添加IT和BLOCKING接收
 *
 * @todo 串口服务会在每个实例注册之后自动启用接收,当前实现为DMA接收
 *       可能还要将此函数修改为extern,使得module可以控制串口的启停
 *
 * @param _instance instance owned by module,模块拥有的串口实例
 */
void USARTServiceInit(USARTInstance *_instance)
{
	__HAL_UART_CLEAR_IDLEFLAG(_instance->usart_handle);
	__HAL_UART_ENABLE_IT(_instance->usart_handle, UART_IT_IDLE);
	SET_BIT(_instance->usart_handle->Instance->CR3, USART_CR3_DMAR);
	HAL_DMAEx_MultiBufferStart(_instance->usart_handle->hdmarx, (uint32_t) & (_instance->usart_handle->Instance->DR), (uint32_t)_instance->Buf_0, (uint32_t)_instance->Buf_1, _instance->recv_buff_size);
}
/**
 * @brief usart 初始化配置结构体赋值
 *
 * @todo 传入USART_Init_Config_s类型结构体，之后传入USARTRegister进行串口实例创建
 *
 *
 * @param
 */
static void USART_Init_Configc_Create(USART_Init_Config_s *conf, uint8_t size, UART_HandleTypeDef *usart_handle, usart_module_callback callback,
									  uint8_t *Buf_0_addr, uint8_t *Buf_1_addr, uint8_t Frame_len)
{

	conf->Buf_0 = Buf_0_addr;
	conf->Buf_1 = Buf_1_addr;
	conf->recv_buff_len = 0;
	conf->recv_buff_size = size;
	conf->recv_Frame_len = Frame_len;
	conf->usart_handle = usart_handle;
	conf->module_callback = callback;
}
