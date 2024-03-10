#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"
#define     DBUS_HUART      huart1
#define  DMA_DBUS_LEN       18
#define  DMA_SUPERCAP_MSG_LEN		28
#define  DMA_MC_MSG_LEN					20	//串口DMA接收缓冲区长度


void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);

#endif

