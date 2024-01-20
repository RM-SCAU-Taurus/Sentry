/**
 *  @file   
 *	@author          
 *	@brief           
 *	@version         
 *	@date            
 *	@param          
 *	@return          
 *	@exception      
 *	@warning         
 *	@remarks         
 *	@note           
 */



#ifndef __UART_DECODE_H__
#define __UART_DECODE_H__

#include "bsp_usart.h"
#include "fifo.h"

void uart_decode_task(void const *argu);


extern fifo_s_t DBUS_fifo;
extern uint8_t DBUS_fifo_buf[5*DMA_DBUS_LEN];
#endif
