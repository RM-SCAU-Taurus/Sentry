#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

typedef struct {
    uint8_t *current_buffer;  // ָ��ǰʹ�õĻ�����
    uint8_t *last_buffer;     // ָ���ϴ�ʹ�õĻ�����
} DoubleBuffer_t,*p_DoubleBuffer_t;

typedef uint8_t (*DoubleBufferArrayPtr)[2]; // ����һ��ָ���ά�����ָ������

#define 	DBUS_HUART    huart1
#define   JUDGE_HUART   huart2
#define 	GYRO_HUART		huart4		//������������ʹ��CAN����ʱ����
#define   VISION_HUART	huart6

#define   Memory0	        0
#define   Memory1	        1
#define  DMA_DBUS_LEN			36
#define  DMA_JUDGE_LEN		122
#define  DMA_VISION_LEN		20
#define  DMA_GYRO_LEN     25

void USER_UART_Init(void);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart);
#endif

