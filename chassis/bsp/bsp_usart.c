#include "bsp_usart.h"
#include "stdlib.h"
#include "string.h"
#include "chassis_task.h"
#include "bsp_powerlimit.h"
#include "remote_msg.h"
#include "status_task.h"
uint8_t dma_dbus_buf[DMA_DBUS_LEN];
 /**
  * @brief  �������ڹ��ܺ���
  * @param 	UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	 if (huart->Instance == USART1) //DBUS����
    {
        wdg_user_set_bit(WDG_BIT_BSP_REMOTE); /* ң����ͨ��״̬��� */
        rc_callback_handler(&rc,dma_dbus_buf);
        memset(dma_dbus_buf, 0, DMA_DBUS_LEN);
        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
    }
}


/**
  * @brief ���ڿ����жϣ�����it.c��ÿ�����ڵ��ж��е��øú�����
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   //�ж��Ƿ��ǿ����ж�
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
		HAL_UART_DMAStop(huart);															//ֹͣ����DMA����
		USER_UART_IDLECallback(huart);                        //���ô��ڹ��ܺ���
	}
}


/**
* @brief ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
*/
void USER_UART_Init(void)
{
	 __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&DBUS_HUART, dma_dbus_buf, DMA_DBUS_LEN);

}
