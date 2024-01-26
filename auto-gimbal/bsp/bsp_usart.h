/**
 *  @file  bsp_usart.h 
 *	@author 	ZGH         
 *	@brief    串口底层驱动       
 *	@version 1.0        
 *	@date    2024-01-19       
 *	@param          
 *	@return          
 *	@exception      
 *	@warning         
 *	@remarks         
 *	@note     1.要在.c文件定义双缓冲区数组，然后传入 uint8_t (*DoubleBufferArrayPtr)[2] 类型指针进行调用。 
							2. 注意在传入形参时，传入的格式和source的正确性	
 */
 
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

#define DEVICE_USART_CNT 2     // 目前分配了2个串口

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)();
/* --------------------------------------------------------------------------------------------------------------------------------- */
// 串口实例结构体,每个module都要包含一个实例.
typedef struct
{
		uint8_t *Buf_0;
		uint8_t *Buf_1;
    uint8_t recv_buff_size;                // DMA设置接收缓冲区的大小
		uint8_t recv_buff_len;                // 模块接收到数据的大小
		uint8_t recv_Frame_len;                // 一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
		uint8_t *Buf_0;
		uint8_t *Buf_1;
    uint8_t recv_buff_size;                // DMA设置接收缓冲区的大小
		uint8_t recv_buff_len;                // 模块接收到数据的大小
		uint8_t recv_Frame_len;                // 一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;


/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

void USARTServiceInit(USARTInstance *_instance);
/* --------------------------------------------------------------------------------------------------------------------------------- */

#define 	DBUS_HUART    huart1
#define   JUDGE_HUART   huart2

#define  DMA_DBUS_LEN			36
#define  Frame_DBUS_LEN		18

#define  DMA_JUDGE_LEN		150
#define  Frame_JUDGE_LEN	122	

void USER_UART_Init(void);
void USER_UART_IDLECallback(UART_HandleTypeDef *huart,uint8_t *buf);
void USER_HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart);
#endif

