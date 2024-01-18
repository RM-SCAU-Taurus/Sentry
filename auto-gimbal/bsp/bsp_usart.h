#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

typedef struct {
    uint16_t receivedBytes;
    uint8_t *current_buffer;  // 指向当前使用的缓冲区
    uint8_t *last_buffer;     // 指向上次使用的缓冲区
} DoubleBuffer_t,*p_DoubleBuffer_t;

#define 	DBUS_HUART    huart1
#define   JUDGE_HUART   huart2
#define 	GYRO_HUART		huart4		//因最新陀螺仪使用CAN，暂时无用
#define   VISION_HUART	huart6

 #define  DMA_DBUS_LEN			36
#define  DMA_JUDGE_LEN		122
#define  DMA_VISION_LEN		20
#define  DMA_GYRO_LEN 25

void USER_UART_Init(void);
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart);
void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
#endif

