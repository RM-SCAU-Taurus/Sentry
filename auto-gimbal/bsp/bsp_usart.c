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
//DBUS����������һ������  it.c����Ҫ��cube�Զ����ɵ��жϺ���ע�͵�������ֻ�ܽ��յ�һ���ֽڡ�ԭ���д��о���
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance== USART1)			//DBUS����
    {
        rc_callback_handler(&rc,DoubleBuffer_dbus.last_buffer);
				memset(DoubleBuffer_dbus.last_buffer,0,DMA_DBUS_LEN);

    }

    else if(huart->Instance== USART2)	//JUDGE����
    {
        judge_data_handler(DoubleBuffer_dbus.last_buffer);
				memset(DoubleBuffer_dbus.last_buffer,0,sizeof(DMA_JUDGE_LEN));
    }
		
}

/**
  * @brief ���ڿ����ж�   ע������it.c��ÿ�����ڵ��ж��е��øú���
  * @param UART_HandleTypeDef *huart
  * @retval ��
  */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
//    if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) //�ж��Ƿ��ǿ����ж�
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(huart);                     //��������жϱ�־�������һֱ���Ͻ����жϣ�
//        HAL_UART_DMAStop(huart);															//ֹͣ����DMA����
//        USER_UART_IDLECallback(huart);                     //���ô��ڹ��ܻص�����
//    }
}


/**
* @brief  ���ڳ�ʼ��:ʹ�ܴ��ڿ����ж�,��������DMA����
* @param  ��
* @retval ��
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
					DoubleBuffer_dbus.current_buffer = dma_dbus_buf[1];   // ��ǰbuf�л���buffer2
					DoubleBuffer_dbus.last_buffer    = dma_dbus_buf[0];   // ��¼�ϴ�bufΪbuffer1
					HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);					// �Ե�ǰbuf�ٴο���DMA����
					__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
				} 
				else if(DoubleBuffer_dbus.current_buffer == dma_dbus_buf[1]) {		
					DoubleBuffer_dbus.current_buffer = dma_dbus_buf[0];   // ��ǰbuf�л���buffer1
					DoubleBuffer_dbus.last_buffer    = dma_dbus_buf[1];   // ��¼�ϴ�bufΪbuffer2
					HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);		// �Ե�ǰbuf�ٴο���DMA����	
					__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	
				}
		}
			if(huart->Instance == USART2){
				if (DoubleBuffer_judge.current_buffer == dma_judge_buf[0]){
					DoubleBuffer_judge.current_buffer = dma_judge_buf[1];   // ��ǰbuf�л���buffer2
					DoubleBuffer_judge.last_buffer    = dma_judge_buf[0];    // ��¼�ϴ�bufΪbuffer1
					HAL_UARTEx_ReceiveToIdle_DMA(&JUDGE_HUART, DoubleBuffer_judge.current_buffer, DMA_JUDGE_LEN);   // �Ե�ǰbuf�ٴο���DMA����
					__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
				} 
				else if(DoubleBuffer_judge.current_buffer == dma_judge_buf[1]){
					DoubleBuffer_judge.current_buffer = dma_judge_buf[0];   // ��ǰbuf�л���buffer1
					DoubleBuffer_judge.last_buffer    = dma_judge_buf[1];   // ��¼�ϴ�bufΪbuffer2
					HAL_UARTEx_ReceiveToIdle_DMA(&JUDGE_HUART, DoubleBuffer_judge.current_buffer, DMA_JUDGE_LEN);		// �Ե�ǰbuf�ٴο���DMA����
					__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
				}
		}
		USER_UART_IDLECallback(huart);  	//���ô��ڹ��ܻص�����
	
}
