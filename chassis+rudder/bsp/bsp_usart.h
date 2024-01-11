#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

#define 	DBUS_HUART    huart1
#define   JUDGE_HUART   huart2
#define 	GYRO_HUART		huart5		//������������ʹ��CAN����ʱ����
#define   VISION_HUART	huart6

#define  DMA_DBUS_LEN			18
#define  DMA_JUDGE_LEN		100
#define  DMA_VISION_LEN		20


void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);

extern uint8_t dma_dbus_buf[DMA_DBUS_LEN];
#endif

