#include "bsp_usart.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_judge.h"
#include "status_task.h"
#include "bsp_JY901.h"

uint8_t	dma_judge_buf[DMA_JUDGE_LEN];
uint8_t dma_dbus_buf[DMA_DBUS_LEN];
uint8_t dma_vision_buf[DMA_VISION_LEN];
uint8_t dma_gyro_buf[DMA_GYRO_LEN];
int test_judge=0;

DoubleBuffer_t DoubleBuffer_dbus;
//DBUS����������һ������  it.c����Ҫ��cube�Զ����ɵ��жϺ���ע�͵�������ֻ�ܽ��յ�һ���ֽڡ�ԭ���д��о���
void USER_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance== USART1)			//DBUS����
    {
        rc_callback_handler(&rc,DoubleBuffer_dbus.last_buffer);
				DoubleBuffer_dbus.receivedBytes = 0;
				memset(DoubleBuffer_dbus.last_buffer,0,DMA_DBUS_LEN);
//			
//				rc_callback_handler(&rc,dma_dbus_buf);
//        HAL_UART_Receive_DMA(huart, dma_dbus_buf, DMA_DBUS_LEN);
//				memset(dma_dbus_buf,0,sizeof(dma_dbus_buf));
    }

//    else if(huart->Instance== USART2)	//JUDGE����
//    {
//        judge_data_handler(dma_judge_buf);
//        HAL_UART_Receive_DMA(huart, dma_judge_buf, DMA_JUDGE_LEN);
//				memset(dma_judge_buf,0,sizeof(dma_judge_buf));
//				test_judge++;
//    }

//    else if(huart->Instance== UART4)	//VISION����
//    {
//        //vision_data_handler(dma_vision_buf);
////			JY901_original_data_read(dma_gyro_buf);
////        HAL_UART_Receive_DMA(huart,dma_gyro_buf,DMA_GYRO_LEN);
//    } 
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
	
		/*********************** DBUS INIT ***************************/
		DoubleBuffer_dbus.current_buffer = DoubleBuffer_dbus.buffer1;
		DoubleBuffer_dbus.last_buffer    = DoubleBuffer_dbus.buffer2;	
//	   __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
//    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);
		/*********************** GYRO INIT ***************************/
//	    __HAL_UART_CLEAR_IDLEFLAG(&GYRO_HUART);
//    __HAL_UART_ENABLE_IT(&GYRO_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&GYRO_HUART, dma_gyro_buf, DMA_GYRO_LEN);
//	
//		/*********************** JUDGE INIT ***************************/
//	
//    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
//    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&JUDGE_HUART, dma_judge_buf, DMA_JUDGE_LEN);
	

//    __HAL_UART_CLEAR_IDLEFLAG(&VISION_HUART);
//    __HAL_UART_ENABLE_IT(&VISION_HUART, UART_IT_IDLE);
//    HAL_UART_Receive_DMA(&VISION_HUART,dma_vision_buf,DMA_VISION_LEN);
}


//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
////		if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)){
//			/* re-use dma+idle to recv */
//			if(huart->Instance == USART1){
//				
//			
//			DoubleBuffer_dbus.receivedBytes = DMA_DBUS_LEN - huart->RxXferCount;
//		
//			if (DoubleBuffer_dbus.current_buffer == DoubleBuffer_dbus.buffer1) 		{
//				
//				DoubleBuffer_dbus.current_buffer = DoubleBuffer_dbus.buffer2;   // ��ǰbuf�л���buffer2
//				DoubleBuffer_dbus.last_buffer    = DoubleBuffer_dbus.buffer1;   // ��¼�ϴ�bufΪbuffer1
//				HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);   // �Ե�ǰbuf�ٴο���DMA����
//			} 
//			else if(DoubleBuffer_dbus.current_buffer == DoubleBuffer_dbus.buffer2) {
//				
//				DoubleBuffer_dbus.current_buffer = DoubleBuffer_dbus.buffer1;   // ��ǰbuf�л���buffer1
//				DoubleBuffer_dbus.last_buffer    = DoubleBuffer_dbus.buffer2;   // ��¼�ϴ�bufΪbuffer2
//				HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);		// �Ե�ǰbuf�ٴο���DMA����
//				
//				
//			}
//		}
//		USER_UART_IDLECallback(huart);  	//���ô��ڹ��ܻص�����
////	}
//}

void HAL_UARTEx_RxEventCallback_dbus(UART_HandleTypeDef *huart, uint16_t Size){
	
			/* re-use dma+idle to recv */
			if(huart->Instance == USART1){
				
			
			DoubleBuffer_dbus.receivedBytes = DMA_DBUS_LEN - huart->RxXferCount;
		
			if (DoubleBuffer_dbus.current_buffer == DoubleBuffer_dbus.buffer1) 		{
				
				DoubleBuffer_dbus.current_buffer = DoubleBuffer_dbus.buffer2;   // ��ǰbuf�л���buffer2
				DoubleBuffer_dbus.last_buffer    = DoubleBuffer_dbus.buffer1;   // ��¼�ϴ�bufΪbuffer1
				HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);   // �Ե�ǰbuf�ٴο���DMA����
			} 
			else if(DoubleBuffer_dbus.current_buffer == DoubleBuffer_dbus.buffer2) {
				
				DoubleBuffer_dbus.current_buffer = DoubleBuffer_dbus.buffer1;   // ��ǰbuf�л���buffer1
				DoubleBuffer_dbus.last_buffer    = DoubleBuffer_dbus.buffer2;   // ��¼�ϴ�bufΪbuffer2
				HAL_UARTEx_ReceiveToIdle_DMA(&DBUS_HUART, DoubleBuffer_dbus.current_buffer, DMA_DBUS_LEN);		// �Ե�ǰbuf�ٴο���DMA����
				
				
			}
		}
		USER_UART_IDLECallback(huart);  	//���ô��ڹ��ܻص�����
	
}
