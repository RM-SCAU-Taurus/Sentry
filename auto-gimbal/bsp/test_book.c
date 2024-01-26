
#ifdef make
/* re-use dma+idle to recv */
		if (huart->Instance == USART1)
		{
			if ((huart->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18) // 接收成功18个字节长度
				{
					rc_usart_instance->module_callback(huart, &dma_dbus_buf[0][0]);
				}

			}
			else
			{
				Memory_change(huart, &rx_msg_dbus, DMA_DBUS_LEN);
				if (rx_msg_dbus.rxlen_rx == 18)							// 接收成功18个字节长度
				{														// 处理遥控器数据
					rc_usart_instance->module_callback(huart, &dma_dbus_buf[1][0]);
				}
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
		
		
		
		
		
		
		
		
		static void UARTX_init(UART_HandleTypeDef *huart, uint16_t LEN)
{
//	__HAL_UART_CLEAR_IDLEFLAG(huart);
//	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
//	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
//	HAL_DMAEx_MultiBufferStart(huart->hdmarx, (uint32_t) & (huart->Instance->DR), (uint32_t)&dma_dbus_buf[0][0], (uint32_t)&dma_dbus_buf[1][0], LEN);
}

#include <stdio.h>

// 使用 typedef 定义指向未知大小一维数组的指针类型
typedef uint8_t (*PtrToBufRow)[];

// 定义函数，接受 PtrToBufRow 类型的参数，以及数组的行数和列数
void processArray(PtrToBufRow ptrToBuf, int numRows, int numCols) {
    // 在函数内部使用指针访问二维数组的元素
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            printf("%u ", ptrToBuf[i][j]);
        }
        printf("\n");
    }
}

int main() {
    // 假设有一个2行N列的二维数组，N是未知的
    uint8_t buf[2][36];

    // 使用 typedef 定义的指针类型
    PtrToBufRow ptrToBuf = buf;

    // 将 buf 数组传递给 processArray 函数，同时传递数组的行数和列数
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