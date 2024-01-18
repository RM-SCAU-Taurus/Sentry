#include "bsp_usart.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_judge.h"
#include "status_task.h"
#include "bsp_JY901.h"


static uint8_t dma_dbus_buf[2][DMA_DBUS_LEN];
static uint8_t dma_judge_buf[2][DMA_JUDGE_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_gyro_buf[DMA_GYRO_LEN];

DoubleBuffer_t DoubleBuffer_dbus;
DoubleBuffer_t DoubleBuffer_judge;
//DBUS串口遗留了一个问题  it.c中需要把cube自动生成的中断函数注释掉，否则只能接收到一个字节。原因有待研究。
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance== USART1)			//DBUS串口
    {
        rc_callback_handler(&rc,DoubleBuffer_dbus.last_buffer);
				memset(DoubleBuffer_dbus.last_buffer,0,DMA_DBUS_LEN);

    }

    else if(huart->Instance== USART2)	//JUDGE串口
    {
        judge_data_handler(DoubleBuffer_dbus.last_buffer);
				memset(DoubleBuffer_dbus.last_buffer,0,sizeof(DMA_JUDGE_LEN));
    }
		
}

/**
  * @brief 串口空闲中断   注：需在it.c中每个串口的中断中调用该函数
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
//    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //判断是否是空闲中断
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
//        HAL_UART_DMAStop(huart);															//停止本次DMA运输
//        USER_UART_IDLECallback(huart);                     //调用串口功能回调函数
//    }
}


/**
* @brief  串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init()
{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1); 
		__HAL_UART_CLEAR_IDLEFLAG(&huart2); 
		/*********************** DBUS INIT ***************************/
		DoubleBuffer_dbus.current_buffer = dma_dbus_buf[0];
		DoubleBuffer_dbus.last_buffer    = dma_dbus_buf[1];	
		HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);
	  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		/*********************** GYRO INIT ***************************/

		/*********************** JUDGE INIT ***************************/	
		DoubleBuffer_judge.current_buffer = dma_judge_buf[0];
		DoubleBuffer_judge.last_buffer    = dma_judge_buf[1];	
    HAL_UART_Receive_DMA(&JUDGE_HUART, DoubleBuffer_judge.current_buffer, DMA_JUDGE_LEN);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

}

void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	
			/* re-use dma+idle to recv */
			if(huart->Instance == USART1){
				if (DoubleBuffer_dbus.current_buffer == dma_dbus_buf[0]) 		{
					DoubleBuffer_dbus.current_buffer = dma_dbus_buf[1];   // 当前buf切换到buffer2
					DoubleBuffer_dbus.last_buffer    = dma_dbus_buf[0];   // 记录上次buf为buffer1
					HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);					// 以当前buf再次开启DMA接收
					__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
				} 
				else if(DoubleBuffer_dbus.current_buffer == dma_dbus_buf[1]) {		
					DoubleBuffer_dbus.current_buffer = dma_dbus_buf[0];   // 当前buf切换到buffer1
					DoubleBuffer_dbus.last_buffer    = dma_dbus_buf[1];   // 记录上次buf为buffer2
					HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);		// 以当前buf再次开启DMA接收	
					__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	
				}
		}
			if(huart->Instance == USART2){
				if (DoubleBuffer_judge.current_buffer == dma_judge_buf[0]){
					DoubleBuffer_judge.current_buffer = dma_judge_buf[1];   // 当前buf切换到buffer2
					DoubleBuffer_judge.last_buffer    = dma_judge_buf[0];    // 记录上次buf为buffer1
					HAL_UARTEx_ReceiveToIdle_DMA(&JUDGE_HUART, DoubleBuffer_judge.current_buffer, DMA_JUDGE_LEN);   // 以当前buf再次开启DMA接收
					__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
				} 
				else if(DoubleBuffer_judge.current_buffer == dma_judge_buf[1]){
					DoubleBuffer_judge.current_buffer = dma_judge_buf[0];   // 当前buf切换到buffer1
					DoubleBuffer_judge.last_buffer    = dma_judge_buf[1];   // 记录上次buf为buffer2
					HAL_UARTEx_ReceiveToIdle_DMA(&JUDGE_HUART, DoubleBuffer_judge.current_buffer, DMA_JUDGE_LEN);		// 以当前buf再次开启DMA接收
					__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
				}
		}
		USER_UART_IDLECallback(huart);  	//调用串口功能回调函数
	
}
