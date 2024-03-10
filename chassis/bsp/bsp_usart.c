#include "bsp_usart.h"
#include "stdlib.h"
#include "string.h"
#include "chassis_task.h"
#include "bsp_powerlimit.h"
#include "remote_msg.h"
#include "status_task.h"
uint8_t dma_dbus_buf[DMA_DBUS_LEN];
 /**
  * @brief  各个串口功能函数
  * @param 	UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == USART1) //DBUS串口
    {
        wdg_user_set_bit(WDG_BIT_BSP_REMOTE); /* 遥控器通信状态检测 */
        rc_callback_handler(&rc,dma_dbus_buf);
        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
}


/**
  * @brief 串口空闲中断（需在it.c中每个串口的中断中调用该函数）
  * @param UART_HandleTypeDef *huart
  * @retval 无
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //判断是否是空闲中断
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //清除空闲中断标志（否则会一直不断进入中断）
		HAL_UART_DMAStop(huart);															//停止本次DMA运输
		USER_UART_IDLECallback(huart);                        //调用串口功能函数
	}
}


/**
* @brief 串口初始化:使能串口空闲中断,开启串口DMA接收
* @param  无
* @retval 无
*/
void USER_UART_Init(void)
{
	 __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);

}

