
#ifdef make
/* re-use dma+idle to recv */
		if (huart->Instance == USART1)
		{
			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18) // ���ճɹ�18���ֽڳ���
				{
					rc_usart_instance->module_callback(huart, &dma_dbus_buf[0][0]);
				}

			}
			else
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18)							// ���ճɹ�18���ֽڳ���
				{														// ����ң��������
					rc_usart_instance->module_callback(huart, &dma_dbus_buf[1][0]);
				}
			}
		}
		
		if (huart->Instance == USART2)
		{
			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				Memory_change(huart, &rx_msg_judge, DMA_JUDGE_LEN);
				if (rx_msg_dbus.rxlen_rx == 122) // ���ճɹ�18���ֽڳ���
				{
					USER_UART_IDLECallback(huart, &dma_judge_buf[0][0]); // Memory_1
				}
			}
			else
			{
				Memory_change(huart, &rx_msg_judge, DMA_JUDGE_LEN);
				if (rx_msg_dbus.rxlen_rx == 122)							// ���ճɹ�18���ֽڳ���
				{														// ����ң��������
					USER_UART_IDLECallback(huart, &dma_judge_buf[1][0]); // Memory_1
				}
			}
		}
		
		
		
		
		
		
		
		
		static void UARTX_init(UART_HandleTypeDef *huart, uint16_t LEN)
{
//	__HAL_UART_CLEAR_IDLEFLAG(huart);
//	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
//	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
//	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t) & (huart->Instance->DR), (uint32_t)&dma_dbus_buf[0][0], (uint32_t)&dma_dbus_buf[1][0], LEN);
}

#include <stdio.h>

// ʹ�� typedef ����ָ��δ֪��Сһά�����ָ������
typedef uint8_t (*PtrToBufRow)[];

// ���庯�������� PtrToBufRow ���͵Ĳ������Լ����������������
void processArray(PtrToBufRow ptrToBuf, int numRows, int numCols) {
    // �ں����ڲ�ʹ��ָ����ʶ�ά�����Ԫ��
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            printf("%u ", ptrToBuf[i][j]);
        }
        printf("\n");
    }
}

int main() {
    // ������һ��2��N�еĶ�ά���飬N��δ֪��
    uint8_t buf[2][36];

    // ʹ�� typedef �����ָ������
    PtrToBufRow ptrToBuf = buf;

    // �� buf ���鴫�ݸ� processArray ������ͬʱ�������������������
    processArray(ptrToBuf, 2, sizeof(buf[0]) / sizeof(buf[0][0]));

    return 0;
}


/**
 * @brief 
 *
 * @todo 
 *       
 *
 * @param 
 */


#endif