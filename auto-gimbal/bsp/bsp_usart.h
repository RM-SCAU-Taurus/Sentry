/**
 *  @file  bsp_usart.h 
 *	@author 	ZGH         
 *	@brief    ���ڵײ�����       
 *	@version 1.0        
 *	@date    2024-01-19       
 *	@param          
 *	@return          
 *	@exception      
 *	@warning         
 *	@remarks         
 *	@note     1.Ҫ��.c�ļ�����˫���������飬Ȼ���� uint8_t (*DoubleBufferArrayPtr)[2] ����ָ����е��á� 
							2. ע���ڴ����β�ʱ������ĸ�ʽ��source����ȷ��	
 */
 
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "usart.h"

#define DEVICE_USART_CNT 2     // Ŀǰ������2������

// ģ��ص�����,���ڽ���Э��
typedef void (*usart_module_callback)();
/* --------------------------------------------------------------------------------------------------------------------------------- */
// ����ʵ���ṹ��,ÿ��module��Ҫ����һ��ʵ��.
typedef struct
{
		uint8_t *Buf_0;
		uint8_t *Buf_1;
    uint8_t recv_buff_size;                // DMA���ý��ջ������Ĵ�С
		uint8_t recv_buff_len;                // ģ����յ����ݵĴ�С
		uint8_t recv_Frame_len;                // һ�����ݵĴ�С
    UART_HandleTypeDef *usart_handle;      // ʵ����Ӧ��usart_handle
    usart_module_callback module_callback; // �����յ������ݵĻص�����
} USARTInstance;

/* usart ��ʼ�����ýṹ�� */
typedef struct
{
		uint8_t *Buf_0;
		uint8_t *Buf_1;
    uint8_t recv_buff_size;                // DMA���ý��ջ������Ĵ�С
		uint8_t recv_buff_len;                // ģ����յ����ݵĴ�С
		uint8_t recv_Frame_len;                // һ�����ݵĴ�С
    UART_HandleTypeDef *usart_handle;      // ʵ����Ӧ��usart_handle
    usart_module_callback module_callback; // �����յ������ݵĻص�����
} USART_Init_Config_s;


/**
 * @brief ע��һ������ʵ��,����һ������ʵ��ָ��
 *
 * @param init_config ���봮�ڳ�ʼ���ṹ��
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

